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

#ifndef COMMAND__INTERFACE_HPP_
#define COMMAND__INTERFACE_HPP_

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>

namespace autoware::control_cmd_gate
{

using autoware_control_msgs::msg::Control;
using autoware_vehicle_msgs::msg::GearCommand;
using autoware_vehicle_msgs::msg::HazardLightsCommand;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;

class CommandOutput
{
public:
  virtual ~CommandOutput() = default;
  virtual void on_control(const Control::ConstSharedPtr msg) = 0;
  virtual void on_gear(const GearCommand::ConstSharedPtr msg) = 0;
  virtual void on_turn_indicators(const TurnIndicatorsCommand::ConstSharedPtr msg) = 0;
  virtual void on_hazard_lights(const HazardLightsCommand::ConstSharedPtr msg) = 0;
};

class CommandInput
{
public:
  explicit CommandInput(CommandOutput * output);
  virtual ~CommandInput() = default;

protected:
  void send_control(const Control::ConstSharedPtr msg);
  void send_gear(const GearCommand::ConstSharedPtr msg);
  void send_turn_indicators(const TurnIndicatorsCommand::ConstSharedPtr msg);
  void send_hazard_lights(const HazardLightsCommand::ConstSharedPtr msg);

private:
  CommandOutput * output_;
};

class CommandBridge : public CommandInput, public CommandOutput
{
public:
  explicit CommandBridge(CommandOutput * output) : CommandInput(output) {}
  void on_control(const Control::ConstSharedPtr msg) override;
  void on_gear(const GearCommand::ConstSharedPtr msg) override;
  void on_turn_indicators(const TurnIndicatorsCommand::ConstSharedPtr msg) override;
  void on_hazard_lights(const HazardLightsCommand::ConstSharedPtr msg) override;
};

}  // namespace autoware::control_cmd_gate

#endif  // COMMAND__INTERFACE_HPP_
