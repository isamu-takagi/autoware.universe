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

#include "subscription.hpp"

#include <string>

namespace autoware::control_cmd_gate
{

CommandSubscription::CommandSubscription(rclcpp::Node & node, const std::string & name)
{
  using std::placeholders::_1;
  const auto control_qos = rclcpp::QoS(5);
  const auto durable_qos = rclcpp::QoS(1).transient_local();
  const auto prefix = "~/inputs/" + name + "/";

  sub_control_ = node.create_subscription<Control>(
    prefix + "control", control_qos, std::bind(&CommandSubscription::send_control, this, _1));
  sub_gear_ = node.create_subscription<GearCommand>(
    prefix + "gear", durable_qos, std::bind(&CommandSubscription::send_gear, this, _1));
  sub_turn_indicators_ = node.create_subscription<TurnIndicatorsCommand>(
    prefix + "turn_indicators", durable_qos,
    std::bind(&CommandSubscription::send_turn_indicators, this, _1));
  sub_hazard_lights_ = node.create_subscription<HazardLightsCommand>(
    prefix + "hazard_lights", durable_qos,
    std::bind(&CommandSubscription::send_hazard_lights, this, _1));
}

}  // namespace autoware::control_cmd_gate
