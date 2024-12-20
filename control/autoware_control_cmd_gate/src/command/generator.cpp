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

CommandGenerator::CommandGenerator(CommandOutput * output, rclcpp::Node & node)
: CommandInput(output)
{
  const auto acceleration = node.declare_parameter<double>("builtin_stop_acceleration");

  control_ = std::make_shared<Control>();
  control_->lateral.steering_tire_rotation_rate = 0.0;
  control_->longitudinal.velocity = 0.0;
  control_->longitudinal.acceleration = acceleration;

  const auto period = rclcpp::Rate(10.0).period();
  clock_ = node.get_clock();
  timer_ = rclcpp::create_timer(&node, clock_, period, [this]() { on_timer(); });
}

void CommandGenerator::on_timer()
{
  const auto stamp = clock_->now();
  control_->stamp = stamp;
  control_->longitudinal.stamp = stamp;
  control_->lateral.stamp = stamp;
  // control_->lateral.steering_tire_angle =
  // control_->lateral.steering_tire_rotation_rate =
  CommandInput::send_control(control_);
}

}  // namespace autoware::control_cmd_gate
