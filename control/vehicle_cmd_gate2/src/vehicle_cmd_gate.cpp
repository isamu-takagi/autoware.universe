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

#include "vehicle_cmd_gate.hpp"

namespace vehicle_cmd_gate2
{

VehicleCmdGate::VehicleCmdGate(const rclcpp::NodeOptions & options)
: Node("vehicle_cmd_gate", options), updater_(this)
{
  const auto input_command_count = 2;

  // Init builtin command generators.
  emergency_command_ = std::make_unique<EmergencyCommand>(*this);
  disengage_command_ = std::make_unique<DisengageCommand>(*this);

  // Init command subscriptions.
  for (size_t i = 1; i <= input_command_count; ++i) {
    const auto ns = "~/input" + std::to_string(i);
    inputs_.push_back(std::make_unique<CommandSubscription>(*this, ns));
  }

  // Init input rate checker.
  updater_.setHardwareID("none");
  for (size_t i = 0; i <= input_command_count; ++i) {
    const auto ns = "input" + std::to_string(i);
    diags_.push_back(std::make_unique<DiagnosticBridge>());
    updater_.add(ns, diags_.back().get(), &DiagnosticBridge::on_diag_task);
  }

  // Init input commands list (0 is builtin emergency).
  std::vector<CommandSender *> commands;
  commands.push_back(emergency_command_.get());
  for (const auto & input : inputs_) commands.push_back(input.get());

  // Bind the rate checker to each input command.
  for (size_t i = 0; i <= input_command_count; ++i) {
    commands[i]->connect(diags_[i].get());
  }

  // Init input command selector and register input commands.
  inputs_selector_ = std::make_unique<CommandSelector>();
  for (const auto & diag : diags_) {
    inputs_selector_->register_input(diag.get());
  }

  // Init engage selector and register selected command.
  engage_selector_ = std::make_unique<CommandSelector>();
  engage_selector_->register_input(disengage_command_.get());
  engage_selector_->register_input(inputs_selector_.get());

  // Init output rate checker.
  output_diag_ = std::make_unique<DiagnosticBridge>();
  engage_selector_->connect(output_diag_.get());

  // Init command publisher.
  output_ = std::make_unique<CommandPublisher>(*this, "~/output");
  output_diag_->connect(output_.get());

  // Set initial command selection.
  inputs_selector_->select(0);
  engage_selector_->select(1);
}

}  // namespace vehicle_cmd_gate2

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vehicle_cmd_gate2::VehicleCmdGate)
