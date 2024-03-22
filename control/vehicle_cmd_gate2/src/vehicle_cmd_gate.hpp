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
#ifndef VEHICLE_CMD_GATE_HPP_
#define VEHICLE_CMD_GATE_HPP_

#include "command/diagnostic.hpp"
#include "command/disengage.hpp"
#include "command/emergency.hpp"
#include "command/publisher.hpp"
#include "command/selector.hpp"
#include "command/subscription.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>

namespace vehicle_cmd_gate2
{

class VehicleCmdGate : public rclcpp::Node
{
public:
  explicit VehicleCmdGate(const rclcpp::NodeOptions & options);

private:
  std::vector<std::unique_ptr<DiagnosticBridge>> diags_;
  std::vector<std::unique_ptr<CommandSubscription>> inputs_;
  std::unique_ptr<CommandSelector> inputs_selector_;
  std::unique_ptr<CommandSelector> engage_selector_;
  std::unique_ptr<CommandPublisher> output_;
  std::unique_ptr<DiagnosticBridge> output_diag_;
  std::unique_ptr<EmergencyCommand> emergency_command_;
  std::unique_ptr<DisengageCommand> disengage_command_;
  diagnostic_updater::Updater updater_;
};

}  // namespace vehicle_cmd_gate2

#endif  // VEHICLE_CMD_GATE_HPP_
