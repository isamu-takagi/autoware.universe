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

#include "control_cmd_gate.hpp"

#include <vector>

namespace autoware::control_cmd_gate
{

ControlCmdGate::ControlCmdGate(const rclcpp::NodeOptions & options)
: Node("control_cmd_gate", options), diag_(this)
{
  RCLCPP_INFO_STREAM(get_logger(), "Test control_cmd_gate");
  diag_.setHardwareID("none");

  const auto params = diagnostic_utils::TimeoutDiag::Params{1.0, 2.0};

  pub_ = std::make_unique<CommandPublisher>(*this);
  abc_ = std::make_unique<CommandDiagnostics>(pub_.get(), diag_, params, *get_clock(), "sample");
  sub_ = std::make_unique<CommandSubscription>(abc_.get(), *this, "sample");

  // Create builtin command.
  /*
  {
    const std::string name = "builtin";

    sources_[build_in_stop] = std::make_unique<EmergencyCommand>(*this, name, diag_);
  }
  */

  // Create command inputs.
  /*
  const auto inputs = declare_parameter<std::vector<std::string>>("inputs");
  for (const auto & input : inputs) {
    sources_[input] = std::make_unique<CommandSubscription>(*this, input, diag_);
  }
  */

  // Select initial command.
  // current_ = sources_.at(build_in_stop).get();
  // current_->select(std::make_unique<CommandPublisher>(*this));

  pub_source_ =
    create_publisher<std_msgs::msg::String>("~/current_source", rclcpp::QoS(1).transient_local());
  sub_source_ = create_subscription<std_msgs::msg::String>(
    "~/select_source", 1,
    std::bind(&ControlCmdGate::on_select_source, this, std::placeholders::_1));
}

void ControlCmdGate::on_select_source(const std_msgs::msg::String & msg)
{
  (void)msg;
  /*
  const auto iter = sources_.find(msg.data);
  if (iter == sources_.end()) {
    RCLCPP_INFO_STREAM(get_logger(), "unknown command source: " << msg.data);
  } else {
    RCLCPP_INFO_STREAM(get_logger(), "changed command source: " << msg.data);
    // auto previous = current_;
    // current_ = iter->second.get();
    // current_->select(*previous);
  }
  */
}

}  // namespace autoware::control_cmd_gate

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::control_cmd_gate::ControlCmdGate)
