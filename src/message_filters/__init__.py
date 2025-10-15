# Copyright 2009, Willow Garage, Inc. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Willow Garage nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""Message Filter Objects."""

from bisect import insort_right
from dataclasses import dataclass
from functools import reduce
import itertools
import threading
from typing import Union

from builtin_interfaces.msg import Time as TimeMsg
import rclpy
from rclpy.clock import ROSClock
from rclpy.duration import Duration
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.time import Time

from typing_extensions import deprecated


class SimpleFilter(object):

    def __init__(self):
        self.callbacks = {}

    def registerCallback(self, cb, *args):
        """
        Register a callback `cb` to be called when this filter has output.

        The filter calls the function ``cb`` with a filter-dependent.

        list of arguments,followed by the call-supplied arguments ``args.``.
        """
        conn = len(self.callbacks)
        self.callbacks[conn] = (cb, args)
        return conn

    def signalMessage(self, *msg):
        for (cb, args) in self.callbacks.values():
            cb(*(msg + args))


class Subscriber(SimpleFilter):
    """
    ROS 2 subscription filter, takes identical arguments as :class:`rclpy.Subscriber`.

    This class acts as a highest-level filter, simply passing messages
    from a ROS 2 subscription through to the filters which have connected
    to it.
    """

    def __init__(self, *args, **kwargs):
        SimpleFilter.__init__(self)
        self.node = args[0]
        self.topic = args[2]
        kwargs.setdefault('qos_profile', 10)
        self.sub = self.node.create_subscription(*args[1:], self.callback, **kwargs)

    def callback(self, msg):
        self.signalMessage(msg)

    def getTopic(self):
        return self.topic

    def __getattr__(self, key):
        """Serve same API as rospy.Subscriber."""
        return self.sub.__getattribute__(key)


class Cache(SimpleFilter):
    """
    Stores a time history of messages.

    Given a stream of messages, the most recent ``cache_size`` messages
    are cached in a ring buffer, from which time intervals of the cache
    can then be retrieved by the client. The ``allow_headerless``
    option specifies whether to allow storing headerless messages with
    current ROS time instead of timestamp. You should avoid this as
    much as you can, since the delays are unpredictable.
    """

    def __init__(self, f, cache_size=1, allow_headerless=False):
        SimpleFilter.__init__(self)
        self.connectInput(f)
        self.cache_size = cache_size
        # Array to store messages
        self.cache_msgs = []
        # Array to store msgs times, auxiliary structure to facilitate
        # sorted insertion
        self.cache_times = []
        # Whether to allow storing headerless messages with current ROS
        # time instead of timestamp.
        self.allow_headerless = allow_headerless

    def connectInput(self, f):
        self.incoming_connection = f.registerCallback(self.add)

    def add(self, msg):
        if not hasattr(msg, 'header') or not hasattr(msg.header, 'stamp'):
            if not self.allow_headerless:
                msg_filters_logger = rclpy.logging.get_logger('message_filters_cache')
                msg_filters_logger.set_level(LoggingSeverity.INFO)
                msg_filters_logger.warn('can not use message filters messages '
                                        'without timestamp infomation when '
                                        '"allow_headerless" is disabled. '
                                        'auto assign ROSTIME to headerless '
                                        'messages once enabling constructor '
                                        'option of "allow_headerless".')

                return

            stamp = ROSClock().now()
        else:
            stamp = msg.header.stamp
            if not hasattr(stamp, 'nanoseconds'):
                stamp = Time.from_msg(stamp)
        # Insert sorted
        self.cache_times.append(stamp)
        self.cache_msgs.append(msg)

        # Implement a ring buffer, discard older if oversized
        if (len(self.cache_msgs) > self.cache_size):
            del self.cache_msgs[0]
            del self.cache_times[0]

        # Signal new input
        self.signalMessage(msg)

    def getInterval(self, from_stamp, to_stamp):
        """Query the current cache content between from_stamp to to_stamp."""
        assert from_stamp <= to_stamp

        return [msg for (msg, time) in zip(self.cache_msgs, self.cache_times)
                if from_stamp <= time <= to_stamp]

    def getElemAfterTime(self, stamp):
        """Return the oldest element after or equal the passed time stamp."""
        newer = [msg for (msg, time) in zip(self.cache_msgs, self.cache_times)
                 if time >= stamp]
        if not newer:
            return None
        return newer[0]

    def getElemBeforeTime(self, stamp):
        """Return the newest element before or equal the passed time stamp."""
        older = [msg for (msg, time) in zip(self.cache_msgs, self.cache_times)
                 if time <= stamp]
        if not older:
            return None
        return older[-1]

    @deprecated('Deprecated in favour of :py:classmethod:Cache.getLatestTime:.')
    def getLastestTime(self):
        """
        Return the newest recorded timestamp.

        Deprecated in favour of :py:classmethod:Cache.getLatestTime:.
        """
        return self.getLatestTime()

    def getLatestTime(self):
        """Return the newest recorded timestamp."""
        if not self.cache_times:
            return None
        return self.cache_times[-1]

    def getOldestTime(self):
        """Return the oldest recorded timestamp."""
        if not self.cache_times:
            return None
        return self.cache_times[0]

    def getLast(self):
        if self.getLatestTime() is None:
            return None
        return self.getElemAfterTime(self.getLatestTime())


class Chain(SimpleFilter):
    """
    Chains a dynamic number of simple filters together.

    Allows retrieval of filters by index after they are added.

    The Chain filter provides a container for simple filters.
    It allows you to store an N-long set of filters inside a single
    structure, making it much easier to manage them.

    Adding filters to the chain is done by adding shared_ptrs of them
    to the filter. They are automatically connected to each other
    and the output of the last filter in the chain is forwarded
    to the callback you've registered with Chain::registerCallback.
    """

    @dataclass
    class FilterInfo:
        message_filter: any
        connection_callback_index: int

    def __init__(self, message_filter=None):
        SimpleFilter.__init__(self)
        if message_filter is not None:
            self.connectInput(message_filter)

        self.incoming_connection = None

        self._message_filters: dict[int, Chain.FilterInfo] = {}

    def connectInput(self, message_filter):
        if self.incoming_connection is not None:
            raise RuntimeError('Already connected')
        self.incoming_connection = message_filter.registerCallback(self.add)

    def add(self, message):
        if self._message_filters:
            self._message_filters[0].message_filter.add(message)
        else:
            self.signalMessage(message)

    def addFilter(self, message_filter):
        new_filter_index = len(self._message_filters)
        last_filter_index = new_filter_index - 1

        self._message_filters[new_filter_index] = Chain.FilterInfo(
            message_filter=message_filter,
            connection_callback_index=message_filter.registerCallback(self.signalMessage),
        )

        if last_filter_index >= 0:
            last_filter = self._message_filters[last_filter_index].message_filter
            callback_index = self._message_filters[last_filter_index].connection_callback_index
            last_filter.callbacks.pop(callback_index)

            self._message_filters[last_filter_index].connection_callback_index = \
                last_filter.registerCallback(message_filter.add)

    def getFilter(self, index: int):
        return self._message_filters[index].message_filter


class TimeSynchronizer(SimpleFilter):
    """
    Synchronizes messages by their timestamps.

    :class:`TimeSynchronizer` synchronizes incoming message filters by the
    timestamps contained in their messages' headers. TimeSynchronizer listens
    on multiple input message filters ``fs``, and invokes the callback when
    it has a collection of messages with matching timestamps.

    The signature of the callback function is:

        def callback(msg1, ... msgN):

    where N is the number of input message filters, and each message is
    the output of the corresponding filter in ``fs``.
    The required ``queue size`` parameter specifies how many sets of
    messages it should store from each input filter (by timestamp)
    while waiting for messages to arrive and complete their "set".
    """

    def __init__(self, fs, queue_size):
        SimpleFilter.__init__(self)
        self.connectInput(fs)
        self.queue_size = queue_size
        self.lock = threading.Lock()

    def connectInput(self, fs):
        self.queues = [{} for f in fs]
        self.input_connections = [
            f.registerCallback(self.add, q, i_q)
            for i_q, (f, q) in enumerate(zip(fs, self.queues))]

    def add(self, msg, my_queue, my_queue_index=None):
        self.lock.acquire()
        stamp = Time.from_msg(msg.header.stamp)
        my_queue[stamp.nanoseconds] = msg
        while len(my_queue) > self.queue_size:
            del my_queue[min(my_queue)]
        # common is the set of timestamps that occur in all queues
        common = reduce(set.intersection, [set(q) for q in self.queues])
        for t in sorted(common):
            # msgs is list of msgs (one from each queue) with stamp t
            msgs = [q[t] for q in self.queues]
            self.signalMessage(*msgs)
            for q in self.queues:
                del q[t]
        self.lock.release()


class ApproximateTimeSynchronizer(TimeSynchronizer):
    """
    Approximately synchronizes messages by their timestamps.

    :class:`ApproximateTimeSynchronizer` synchronizes incoming message filters
    by the timestamps contained in their messages' headers. The API is the same
    as TimeSynchronizer except for an extra `slop` parameter in the constructor
    that defines the delay (in seconds) with which messages can be synchronized.
    The ``queue_offset`` option allow to have temporal offset between subscribers
    , define as a list of offset int in nanoseconds.
    The ``allow_headerless`` option specifies whether to allow storing
    headerless messages with current ROS time instead of timestamp. You should
    avoid this as much as you can, since the delays are unpredictable.
    The ```sync_arrival_time``` option enables synchronizing incoming messages
    with the arrival ROS time instead of the message timestamp. You should
    avoid this as much as you can, since the delays are unpredictable.
    """

    def __init__(self, fs, queue_size, slop,
                 queue_offset=False,
                 allow_headerless=False,
                 sync_arrival_time=False):
        TimeSynchronizer.__init__(self, fs, queue_size)
        self.slop = Duration(seconds=slop)
        self.allow_headerless = allow_headerless
        self.queue_offset = queue_offset
        self.sync_arrival_time = sync_arrival_time

    def add(self, msg, my_queue, my_queue_index=None):
        if not hasattr(msg, 'header') or not hasattr(msg.header, 'stamp') or \
                self.sync_arrival_time:
            if not self.allow_headerless and not self.sync_arrival_time:
                msg_filters_logger = rclpy.logging.get_logger('message_filters_approx')
                msg_filters_logger.set_level(LoggingSeverity.INFO)
                msg_filters_logger.warn('can not use message filters messages '
                                        'without timestamp infomation when '
                                        '"allow_headerless" is disabled. '
                                        'auto assign ROSTIME to headerless '
                                        'messages once enabling constructor '
                                        'option of "allow_headerless".')
                return

            stamp = ROSClock().now()
        else:
            stamp = msg.header.stamp
            if not hasattr(stamp, 'nanoseconds'):
                stamp = Time.from_msg(stamp)
            # print(stamp)
        new_timestamp = stamp.nanoseconds
        if my_queue_index is not None and self.queue_offset:
            new_timestamp -= self.queue_offset[my_queue_index]
        self.lock.acquire()
        my_queue[new_timestamp] = msg
        while len(my_queue) > self.queue_size:
            del my_queue[min(my_queue)]
        # self.queues = [topic_0 {stamp: msg}, topic_1 {stamp: msg}, ...]
        if my_queue_index is None:
            search_queues = self.queues
        else:
            search_queues = self.queues[:my_queue_index] + \
                self.queues[my_queue_index+1:]
        # sort and leave only reasonable stamps for synchronization
        stamps = []
        for queue in search_queues:
            topic_stamps = []
            for s in queue:
                stamp_delta = Duration(nanoseconds=abs(s - new_timestamp))
                if stamp_delta > self.slop:
                    continue  # far over the slop
                topic_stamps.append(((Time(nanoseconds=s,
                                           clock_type=stamp.clock_type)),
                                    stamp_delta))
            if not topic_stamps:
                self.lock.release()
                return
            topic_stamps = sorted(topic_stamps, key=lambda x: x[1])
            stamps.append(topic_stamps)
        for vv in itertools.product(*[list(zip(*s))[0] for s in stamps]):
            vv = list(vv)
            # insert the new message
            if my_queue_index is not None:
                vv.insert(my_queue_index, stamp)
            qt = list(zip(self.queues, vv))
            if (((max(vv) - min(vv)) < self.slop) and
               (len([1 for q, t in qt if t.nanoseconds not in q]) == 0)):
                msgs = [q[t.nanoseconds] for q, t in qt]
                self.signalMessage(*msgs)
                for q, t in qt:
                    del q[t.nanoseconds]
                break  # fast finish after the synchronization
        self.lock.release()


class TimeSequencer(SimpleFilter):
    """
    Sequences messages based on the timestamp of their header.

    At construction, the TimeSequencer takes a duration 'delay' which specifies
    how long to queue up messages to provide a time sequencing over them.
    As messages arrive, they are sorted according to their timestamps.
    A callback for a message is never invoked until the messages' timestamp is
    out of date by at least the delay. However, for all messages which are out of
    date by at least delay, their callbacks are invoked in temporal order.
    If a message arrives from a time prior to a message which has already had its
    callback invoked, it is thrown away.
    """

    def __init__(
        self,
        input_filter: SimpleFilter,
        delay: Union[Duration, float],
        update_rate: Union[Duration, float],
        queue_size: int,
        node: Node,
        msg_stamp_attr: str = 'header.stamp',
    ):
        """
        Construct a TimeSequencer filter for a subscriber.

        Args:
        ----
        input_filter (SimpleFilter): The input filter to connect to.
            Typically a Subscriber.
        delay (Duration | float): The delay (in seconds) to wait for
            messages to arrive before dispatching them.
        update_rate (Duration | float): The rate at which to check for
            messages that are ready to be dispatched.
        queue_size (int): The maximum number of messages to store. Set 0
            for no limit.
        node (Node): The node to create the timer on.
        msg_stamp_attr (str, optional): The attribute to use for retrieving
            the timestamp from the message. Should point to a
            builtin_interfaces.msg.Time field. Defaults to "header.stamp".

        """
        super().__init__()
        if not isinstance(delay, Duration):
            delay = Duration(seconds=delay)
        if not isinstance(update_rate, Duration):
            update_rate = Duration(seconds=update_rate)
        self.delay: float = delay
        self.update_rate: float = update_rate
        self.queue_size: int = queue_size
        self.lock = threading.Lock()
        self.messages = []
        self.last_time: Time = Time()
        self.node: Node = node
        self.msg_stamp_attrs = msg_stamp_attr.split('.')
        self.update_timer = self.node.create_timer(
            self.update_rate.nanoseconds / 1e9, self._dispatch
        )
        self.incoming_connection = None
        if input_filter is not None:
            self.connectInput(input_filter)

    def _getMsgStampAttr(self, msg):
        obj = msg
        for attr in self.msg_stamp_attrs:
            if not hasattr(obj, attr):
                return None
            obj = getattr(obj, attr)
        return obj

    def connectInput(self, input_filter: SimpleFilter):
        if self.incoming_connection is not None:
            raise RuntimeError('Already connected to an input filter.')
        self.incoming_connection = input_filter.registerCallback(self._add)

    def _add(self, msg):
        with self.lock:
            stamp = self._getStamp(msg)
            if stamp is None:
                return
            if stamp.nanoseconds < self.last_time.nanoseconds:
                return
            # Insert msg into messages in sorted order
            insort_right(self.messages, (stamp, msg))
            # If queue_size is exceeded, remove the earliest message
            if self.queue_size != 0 and len(self.messages) > self.queue_size:
                del self.messages[0]

    def _getStamp(self, msg):
        stamp = self._getMsgStampAttr(msg)
        if stamp is not None:
            if not isinstance(stamp, TimeMsg):
                raise TypeError(
                    f'Expected {TimeMsg}, got {type(stamp)} in msg attribute '
                    f"{'.'.join(self.msg_stamp_attrs)}"
                )
            stamp = Time.from_msg(stamp)
            return stamp
        else:
            self.node.get_logger().warn(
                'Cannot use message without timestamp; discarding message.'
            )
            return None

    def _dispatch(self):
        to_call = []
        with self.lock:
            while self.messages:
                stamp, msg = self.messages[0]
                if stamp + self.delay <= self.node.get_clock().now():
                    self.last_time = stamp
                    # Remove message from messages
                    self.messages.pop(0)
                    to_call.append(msg)
                else:
                    break
        for msg in to_call:
            self.signalMessage(msg)

    def shutdown(self):
        """Clean up the TimeSequencer."""
        self.update_timer.cancel()
        self.node.destroy_timer(self.update_timer)
