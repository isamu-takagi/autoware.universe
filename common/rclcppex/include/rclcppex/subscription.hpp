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

#ifndef RCLCPPEX__SUBSCRIPTION_HPP_
#define RCLCPPEX__SUBSCRIPTION_HPP_

#include "rclcppex/topic_rate_monitor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace rclcppex
{

template <class MessageT>
class Subscription : public TopicRateMonitor
{
public:
  using WrapTypeSharedPtr = typename rclcpp::Subscription<MessageT>::SharedPtr;
  using CallbackType = std::function<void(const typename MessageT::ConstSharedPtr msg)>;
  RCLCPP_SMART_PTR_DEFINITIONS(Subscription<MessageT>)

  Subscription(rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos)
  : TopicRateMonitor(node->get_clock())
  {
    sub_ = node->create_subscription<MessageT>(
      topic_name, qos, std::bind(&Subscription::on_message, this, std::placeholders::_1));
  }

  void on_message(const typename MessageT::ConstSharedPtr msg)
  {
    insert_queue();
    for (const auto & callback : callbacks_) {
      callback(msg);
    }
  }

  void add_callback(CallbackType && callback) { callbacks_.push_back(callback); }

private:
  WrapTypeSharedPtr sub_;
  std::vector<CallbackType> callbacks_;
};

}  // namespace rclcppex

#endif  // RCLCPPEX__SUBSCRIPTION_HPP_
