// Copyright 2024 The Autoware Contributors
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

#ifndef DIAGNOSTIC_GRAPH_UTILS__SUBSCRIPTION_HPP_
#define DIAGNOSTIC_GRAPH_UTILS__SUBSCRIPTION_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/msg/diag_graph_status.hpp>
#include <tier4_system_msgs/msg/diag_graph_struct.hpp>

namespace diagnostic_graph_utils
{

class DiagnosticGraphSubscription
{
public:
  DiagnosticGraphSubscription(rclcpp::Node & node, size_t depth);

private:
  using DiagGraphStruct = tier4_system_msgs::msg::DiagGraphStruct;
  using DiagGraphStatus = tier4_system_msgs::msg::DiagGraphStatus;
  void on_struct(const DiagGraphStruct & msg);
  void on_status(const DiagGraphStatus & msg);
  rclcpp::Subscription<DiagGraphStruct>::SharedPtr sub_struct_;
  rclcpp::Subscription<DiagGraphStatus>::SharedPtr sub_status_;
};

}  // namespace diagnostic_graph_utils

#endif  // DIAGNOSTIC_GRAPH_UTILS__SUBSCRIPTION_HPP_
