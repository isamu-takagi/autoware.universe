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

#ifndef RCLCPPEX__PUBLISHER_HPP_
#define RCLCPPEX__PUBLISHER_HPP_

#include "rclcppex/topic_rate_monitor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace rclcppex
{

template <class MessageT>
class Publisher : public TopicRateMonitor
{
public:
  using WrapTypeSharedPtr = typename rclcpp::Publisher<MessageT>::SharedPtr;
  RCLCPP_SMART_PTR_DEFINITIONS(Publisher<MessageT>)

  Publisher(rclcpp::Node * node, const std::string & topic_name, const rclcpp::QoS & qos)
  : TopicRateMonitor(node->get_clock())
  {
    pub_ = node->create_publisher<MessageT>(topic_name, qos);
  }

  void publish(const MessageT & msg)
  {
    insert_queue();
    pub_->publish(msg);
  }

private:
  WrapTypeSharedPtr pub_;
};

}  // namespace rclcppex

#endif  // RCLCPPEX__PUBLISHER_HPP_
