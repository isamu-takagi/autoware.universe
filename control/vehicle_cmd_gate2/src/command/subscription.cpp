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

namespace vehicle_cmd_gate2
{

CommandSubscription::CommandSubscription(rclcpp::Node & node, const std::string & ns)
{
  using std::placeholders::_1;
  sub_control_ = node.create_subscription<AckermannControlCommand>(
    ns + "/control", 1, std::bind(&CommandSubscription::send_control, this, _1));
  sub_gear_ = node.create_subscription<GearCommand>(
    ns + "/gear", 1, std::bind(&CommandSubscription::send_gear, this, _1));
  sub_turn_indicators_ = node.create_subscription<TurnIndicatorsCommand>(
    ns + "/turn_indicators", 1, std::bind(&CommandSubscription::send_turn_indicators, this, _1));
  sub_hazard_lights_ = node.create_subscription<HazardLightsCommand>(
    ns + "/hazard_lights", 1, std::bind(&CommandSubscription::send_hazard_lights, this, _1));
}

}  // namespace vehicle_cmd_gate2
