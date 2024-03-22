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

#include "publisher.hpp"

namespace vehicle_cmd_gate2
{

CommandPublisher::CommandPublisher(rclcpp::Node & node, const std::string & ns)
{
  const auto durable_qos = rclcpp::QoS(1).transient_local();
  pub_control_ = node.create_publisher<AckermannControlCommand>(ns + "/control", durable_qos);
  pub_gear_ = node.create_publisher<GearCommand>(ns + "/gear", durable_qos);
  pub_turn_indicators_ =
    node.create_publisher<TurnIndicatorsCommand>(ns + "/turn_indicators", durable_qos);
  pub_hazard_lights_ =
    node.create_publisher<HazardLightsCommand>(ns + "/hazard_lights", durable_qos);
}

void CommandPublisher::on_control(const AckermannControlCommand::ConstSharedPtr msg)
{
  pub_control_->publish(*msg);
}

void CommandPublisher::on_gear(const GearCommand::ConstSharedPtr msg)
{
  pub_gear_->publish(*msg);
}

void CommandPublisher::on_turn_indicators(const TurnIndicatorsCommand::ConstSharedPtr msg)
{
  pub_turn_indicators_->publish(*msg);
}

void CommandPublisher::on_hazard_lights(const HazardLightsCommand::ConstSharedPtr msg)
{
  pub_hazard_lights_->publish(*msg);
}

}  // namespace vehicle_cmd_gate2
