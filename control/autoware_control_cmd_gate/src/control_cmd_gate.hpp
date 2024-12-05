//  Copyright 2023 The Autoware Contributors
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#ifndef CONTROL_CMD_GATE_HPP_
#define CONTROL_CMD_GATE_HPP_

#include "command/diagnostics.hpp"
#include "command/interface.hpp"
#include "command/publisher.hpp"
#include "command/subscription.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <unordered_map>

// DEBUG
#include <std_msgs/msg/string.hpp>

namespace autoware::control_cmd_gate
{

class ControlCmdGate : public rclcpp::Node
{
public:
  explicit ControlCmdGate(const rclcpp::NodeOptions & options);

private:
  static constexpr char build_in_stop[] = "builtin";

  void on_select_source(const std_msgs::msg::String & msg);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_source_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_source_;

  diagnostic_updater::Updater diag_;
  std::unordered_map<std::string, std::unique_ptr<CommandSubscription>> subs_;
  std::unique_ptr<CommandPublisher> pub_;

  // TEMP
  std::unique_ptr<CommandSubscription> sub_;
  std::unique_ptr<CommandDiagnostics> abc_;
};

}  // namespace autoware::control_cmd_gate

#endif  // CONTROL_CMD_GATE_HPP_
