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

#include "command/generator.hpp"
#include "command/publisher.hpp"
#include "command/subscription.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::control_cmd_gate
{

ControlCmdGate::ControlCmdGate(const rclcpp::NodeOptions & options)
: Node("control_cmd_gate", options), diag_(this)
{
  selector_ = std::make_unique<CommandSelector>();
  diag_.setHardwareID("none");

  // const auto params = diagnostic_utils::TimeoutDiag::Params{1.0, 2.0};
  const auto inputs = declare_parameter<std::vector<std::string>>("inputs");
  if (std::find(inputs.begin(), inputs.end(), builtin) != inputs.end()) {
    throw std::invalid_argument("input name '" + builtin + "' is reserved");
  }

  // Create input command sources.
  for (const auto & input : inputs) {
    auto source = std::make_unique<CommandSubscription>(*this, input);
    // auto filter = std::make_unique<CommandDiagnostics>(diag_, params, *get_clock(), input);
    // source->add_filter(std::move(filter));
    selector_->add_source(input, std::move(source));
  }

  // Create builtin command source.
  {
    auto source = std::make_unique<CommandGenerator>(*this);
    // auto filter = std::make_unique<CommandDiagnostics>(diag_, params, *get_clock(), builtin);
    // source->add_filter(std::move(filter));
    selector_->add_source(builtin, std::move(source));
  }

  // Create command output.
  {
    auto output = std::make_unique<CommandPublisher>(*this);
    selector_->set_output(std::move(output));
  }

  // Select initial source.
  selector_->select(builtin);

  // Create selector interface.
  pub_source_ =
    create_publisher<std_msgs::msg::String>("~/current_source", rclcpp::QoS(1).transient_local());
  sub_source_ = create_subscription<std_msgs::msg::String>(
    "~/select_source", 1,
    std::bind(&ControlCmdGate::on_select_source, this, std::placeholders::_1));
}

void ControlCmdGate::on_select_source(const std_msgs::msg::String & msg)
{
  const auto result = selector_->select(msg.data);
  if (result) {
    RCLCPP_INFO_STREAM(get_logger(), "changed command source: " << msg.data);
  } else {
    RCLCPP_INFO_STREAM(get_logger(), "unknown command source: " << msg.data);
  }
}

}  // namespace autoware::control_cmd_gate

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::control_cmd_gate::ControlCmdGate)
