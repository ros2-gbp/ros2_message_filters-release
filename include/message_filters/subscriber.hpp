// Copyright 2009, Willow Garage, Inc. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef MESSAGE_FILTERS__SUBSCRIBER_HPP_
#define MESSAGE_FILTERS__SUBSCRIBER_HPP_

#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>

#include <rclcpp/create_subscription.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/rclcpp.hpp>

#include "message_filters/connection.hpp"
#include "message_filters/simple_filter.hpp"

namespace message_filters
{

class SubscriberBase
{
public:
  using NodeParametersInterface = rclcpp::node_interfaces::NodeParametersInterface;
  using NodeTopicsInterface = rclcpp::node_interfaces::NodeTopicsInterface;

  using RequiredInterfaces = rclcpp::node_interfaces::NodeInterfaces<NodeParametersInterface,
      NodeTopicsInterface>;

  SubscriberBase() {}

  virtual ~SubscriberBase() = default;
  /**
   * \brief Subscribe to a topic.
   *
   * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
   *
   * \param node_interfaces The NodeInterfaces to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param qos (optional) The rmw qos profile to use to subscribe
   */
  virtual void subscribe(
    RequiredInterfaces node_interfaces, const std::string & topic,
    const rclcpp::QoS & qos) = 0;

  /**
   * \brief Subscribe to a topic.
   *
   * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
   *
   * \param node_interfaces The NodeInterfaces to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param qos The rmw qos profile to use to subscribe.
   * \param options The subscription options to use to subscribe.
   */
  virtual void subscribe(
    RequiredInterfaces node_interfaces,
    const std::string & topic,
    const rclcpp::QoS & qos,
    rclcpp::SubscriptionOptions options) = 0;

  /**
   * \brief Re-subscribe to a topic.  Only works if this subscriber has previously been subscribed to a topic.
   */
  virtual void subscribe() = 0;
  /**
   * \brief Force immediate unsubscription of this subscriber from its topic
   */
  virtual void unsubscribe() = 0;
};

/**
 * \brief ROS subscription filter.
 *
 * This class acts as a highest-level filter, simply passing messages from a ROS subscription through to the
 * filters which have connected to it.
 *
 * When this object is destroyed it will unsubscribe from the ROS subscription.
 *
 * The Subscriber object is templated on the type of message being subscribed to.
 *
 * \section connections CONNECTIONS
 *
 * Subscriber has no input connection.
 *
 * The output connection for the Subscriber object is the same signature as for rclcpp subscription callbacks, ie.
\verbatim
void callback(const std::shared_ptr<M const> &);
\endverbatim
 */

template<typename M, bool is_adapter = rclcpp::is_type_adapter<M>::value>
struct message_type;

template<typename M>
struct message_type<M, true>
{
  using type = typename M::custom_type;
};

template<typename M>
struct message_type<M, false>
{
  using type = M;
};

template<typename M>
using message_type_t = typename message_type<M>::type;

template<class M>
class Subscriber
  : public SubscriberBase,
  public SimpleFilter<message_type_t<M>>
{
public:
  typedef message_type_t<M> MessageType;
  typedef MessageEvent<MessageType const> EventType;

  /**
   * \brief Constructor
   *
   * See the rclcpp::Node::subscribe() variants for more information on the parameters
   *
   * \param node The NodeInterfaces to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param qos (optional) The rmw qos profile to use to subscribe
   */
  Subscriber(
    RequiredInterfaces node_interfaces, const std::string & topic,
    const rclcpp::QoS & qos)
  {
    subscribe(node_interfaces, topic, qos);
  }

  template<typename NodeT>
  Subscriber(
    std::shared_ptr<NodeT> node,
    const std::string & topic,
    const rclcpp::QoS & qos)
  : Subscriber(*node, topic, qos) {}

  template<typename NodeT>
  Subscriber(
    NodeT * node,
    const std::string & topic,
    const rclcpp::QoS & qos)
  : Subscriber(*node, topic, qos) {}

  /**
   * \brief Constructor
   *
   * See the rclcpp::Node::subscribe() variants for more information on the parameters
   *
   * \param node The NodeInterfaces to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param qos The rmw qos profile to use to subscribe.
   * \param options The subscription options to use to subscribe.
   */
  Subscriber(
    RequiredInterfaces node_interfaces,
    const std::string & topic,
    const rclcpp::QoS & qos,
    rclcpp::SubscriptionOptions options)
  {
    subscribe(node_interfaces, topic, qos, options);
  }


  template<typename NodeT>
  Subscriber(
    std::shared_ptr<NodeT> node,
    const std::string & topic,
    const rclcpp::QoS & qos,
    rclcpp::SubscriptionOptions options)
  : Subscriber(*node, topic, qos, options) {}

  template<typename NodeT>
  Subscriber(
    NodeT * node,
    const std::string & topic,
    const rclcpp::QoS & qos,
    rclcpp::SubscriptionOptions options)
  : Subscriber(*node, topic, qos, options) {}

  /**
   * \brief Empty constructor, use subscribe() to subscribe to a topic
   */
  Subscriber() = default;

  ~Subscriber()
  {
    unsubscribe();
  }

  /**
   * \brief Subscribe to a topic.
   *
   * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
   *
   * \param node The NodeInterfaces to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param qos (optional) The rmw qos profile to use to subscribe
   */
  void subscribe(
    RequiredInterfaces node_interfaces, const std::string & topic,
    const rclcpp::QoS & qos) override
  {
    subscribe(node_interfaces, topic, qos, rclcpp::SubscriptionOptions());
  }

  template<typename NodeT>
  void subscribe(
    NodeT * node, const std::string & topic,
    const rclcpp::QoS & qos)
  {
    subscribe(*node, topic, qos, rclcpp::SubscriptionOptions());
  }

  template<typename NodeT>
  void subscribe(
    std::shared_ptr<NodeT> node, const std::string & topic,
    const rclcpp::QoS & qos)
  {
    subscribe(*node, topic, qos, rclcpp::SubscriptionOptions());
  }

  /**
   * \brief Subscribe to a topic.
   *
   * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
   *
   * \param node The NodeInterfaces to use to subscribe.
   * \param topic The topic to subscribe to.
   * \param qos The rmw qos profile to use to subscribe.
   * \param options The subscription options to use to subscribe.
   */
  void subscribe(
    RequiredInterfaces node_interfaces,
    const std::string & topic,
    const rclcpp::QoS & qos,
    rclcpp::SubscriptionOptions options) override
  {
    unsubscribe();

    if (!topic.empty()) {
      topic_ = topic;
      qos_ = qos;
      options_ = options;

      // Note: temporary variables are solely needed because the involved submodules are
      //       inconsistent about the passing of shared_ptr. While the node_interfaces return
      //       by value, the create_subscription function expects the shared_ptr to be passed
      //       via reference...
      auto parameters_interface = node_interfaces.template get<NodeParametersInterface>();
      auto topics_interface = node_interfaces.template get<NodeTopicsInterface>();

      sub_ = rclcpp::create_subscription<MessageType>(parameters_interface,
                                                      topics_interface,
                                                      topic, qos,
          [this](const std::shared_ptr<MessageType const> msg) {
            this->cb(EventType(msg));
          }, options);

      node_interfaces_ = node_interfaces;
    }
  }

  template<typename NodeT>
  void subscribe(
    NodeT * node, const std::string & topic,
    const rclcpp::QoS & qos,
    rclcpp::SubscriptionOptions options)
  {
    subscribe(*node, topic, qos, options);
  }

  template<typename NodeT>
  void subscribe(
    std::shared_ptr<NodeT> node, const std::string & topic,
    const rclcpp::QoS & qos,
    rclcpp::SubscriptionOptions options)
  {
    subscribe(*node, topic, qos, options);
  }

  /**
   * \brief Re-subscribe to a topic.  Only works if this subscriber has previously been subscribed to a topic.
   */
  void subscribe() override
  {
    if (!topic_.empty()) {
      subscribe(node_interfaces_, topic_, qos_, options_);
    }
  }

  /**
   * \brief Force immediate unsubscription of this subscriber from its topic
   */
  void unsubscribe() override
  {
    sub_.reset();
  }

  std::string getTopic() const
  {
    return this->topic_;
  }

  /**
   * \brief Returns the internal rclcpp::Subscription<M>::SharedPtr object
   */
  const typename rclcpp::Subscription<M>::SharedPtr getSubscriber() const {return sub_;}

  /**
   * \brief Does nothing.  Provided so that Subscriber may be used in a message_filters::Chain
   */
  template<typename F>
  void connectInput(F & f)
  {
    (void)f;
  }

  /**
   * \brief Does nothing.  Provided so that Subscriber may be used in a message_filters::Chain
   */
  void add(const EventType & e)
  {
    (void)e;
  }

private:
  void cb(const EventType & e)
  {
    this->signalMessage(e);
  }

  typename rclcpp::Subscription<M>::SharedPtr sub_;

  RequiredInterfaces node_interfaces_;

  std::string topic_;
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  rclcpp::SubscriptionOptions options_;
};

}  // namespace message_filters

#endif  // MESSAGE_FILTERS__SUBSCRIBER_HPP_
