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

#include "generator.hpp"

#include <memory>

namespace autoware::control_cmd_gate
{

CommandGenerator::CommandGenerator(rclcpp::Node & node)
{
  acceleration_ = node.declare_parameter<double>("builtin_stop_acceleration");

  const auto period = rclcpp::Rate(10.0).period();
  clock_ = node.get_clock();
  timer_ = rclcpp::create_timer(&node, clock_, period, [this]() { on_timer(); });
}

void CommandGenerator::on_timer()
{
  const auto stamp = clock_->now();
  const auto control = std::make_shared<Control>();
  control->stamp = stamp;
  control->longitudinal.stamp = stamp;
  control->longitudinal.velocity = 0.0;
  control->longitudinal.acceleration = acceleration_;
  control->lateral.stamp = stamp;
  control->lateral.steering_tire_angle = 0.0;
  control->lateral.steering_tire_rotation_rate = 0.0;
  CommandInput::send_control(control);
}

void CommandGenerator::resend_last_command()
{
  const auto stamp = clock_->now();
  const auto gear = std::make_shared<GearCommand>();
  const auto turn_indicators = std::make_shared<TurnIndicatorsCommand>();
  const auto hazard_lights = std::make_shared<HazardLightsCommand>();

  gear->stamp = stamp;
  gear->command = GearCommand::DRIVE;  // TODO(Takagi, Isamu): Use last_gear_

  turn_indicators->stamp = stamp;
  turn_indicators->command = TurnIndicatorsCommand::DISABLE;

  hazard_lights->stamp = stamp;
  hazard_lights->command = HazardLightsCommand::ENABLE;

  // NOTE: Control does not need to be resent because it is sent periodically.
  send_gear(gear);
  send_turn_indicators(turn_indicators);
  send_hazard_lights(hazard_lights);
}

}  // namespace autoware::control_cmd_gate
