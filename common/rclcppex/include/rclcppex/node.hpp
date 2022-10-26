// Copyright 2022 Takagi, Isamu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RCLCPPEX__NODE_HPP_
#define RCLCPPEX__NODE_HPP_

#include "rclcppex/publisher.hpp"
#include "rclcppex/subscription.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>

namespace rclcppex
{

class Node : public rclcpp::Node
{
public:
  using rclcpp::Node::Node;

  template <typename MessageT>
  std::shared_ptr<rclcppex::Publisher<MessageT>> create_publisher(
    const std::string & topic_name, const rclcpp::QoS & qos)
  {
    return rclcppex::Publisher<MessageT>::make_shared(this, topic_name, qos);
  }

  template <typename MessageT, typename CallbackT>
  std::shared_ptr<rclcppex::Subscription<MessageT>> create_subscription(
    const std::string & topic_name, const rclcpp::QoS & qos, CallbackT && callback)
  {
    auto sub = rclcppex::Subscription<MessageT>::make_shared(this, topic_name, qos);
    sub->add_callback(std::forward<CallbackT>(callback));
    return sub;
  }

private:
  // Prohibit direct access to original methods.
  using rclcpp::Node::create_client;
  using rclcpp::Node::create_publisher;
  using rclcpp::Node::create_service;
  using rclcpp::Node::create_subscription;
};

}  // namespace rclcppex

#endif  // RCLCPPEX__NODE_HPP_
