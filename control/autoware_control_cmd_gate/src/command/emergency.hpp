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

#ifndef COMMAND__EMERGENCY_HPP_
#define COMMAND__EMERGENCY_HPP_

#include "interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware::control_cmd_gate
{

class EmergencyCommand : public CommandSource
{
public:
  EmergencyCommand(
    rclcpp::Node & node, const std::string & name, diagnostic_updater::Updater & diag);

private:
  void on_timer();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock::SharedPtr clock_;

  Control::SharedPtr control_;
  GearCommand gear_;
  TurnIndicatorsCommand turn_indicators_;
  HazardLightsCommand hazard_lights_;
};

}  // namespace autoware::control_cmd_gate

#endif  // COMMAND__EMERGENCY_HPP_
