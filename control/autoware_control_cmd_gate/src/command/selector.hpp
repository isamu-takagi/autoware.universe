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

#ifndef COMMAND__SELECTOR_HPP_
#define COMMAND__SELECTOR_HPP_

#include "../diag/timeout.hpp"

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <memory>
#include <string>

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

class CommandSource
{
public:
  explicit CommandSource(const std::string & name, diagnostic_updater::Updater & diag);
  virtual ~CommandSource() = default;

  void select(std::unique_ptr<CommandOutput> && output);
  void select(CommandSource & source);

protected:
  void on_control(const Control::ConstSharedPtr msg);
  void on_gear(const GearCommand::ConstSharedPtr msg);
  void on_turn_indicators(const TurnIndicatorsCommand::ConstSharedPtr msg);
  void on_hazard_lights(const HazardLightsCommand::ConstSharedPtr msg);

private:
  std::unique_ptr<CommandOutput> output_;
  TimeoutDiag timeout_diag_;
  GearCommand::ConstSharedPtr gear_;
  TurnIndicatorsCommand::ConstSharedPtr turn_indicators_;
  HazardLightsCommand::ConstSharedPtr hazard_lights_;
};

}  // namespace autoware::control_cmd_gate

#endif  // COMMAND__SELECTOR_HPP_
