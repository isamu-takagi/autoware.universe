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

#ifndef COMMAND__SUBSCRIPTION_HPP_
#define COMMAND__SUBSCRIPTION_HPP_

#include "interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace vehicle_cmd_gate2
{

class CommandSubscription : public CommandSender
{
public:
  CommandSubscription(rclcpp::Node & node, const std::string & ns);

private:
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr sub_control_;
  rclcpp::Subscription<GearCommand>::SharedPtr sub_gear_;
  rclcpp::Subscription<TurnIndicatorsCommand>::SharedPtr sub_turn_indicators_;
  rclcpp::Subscription<HazardLightsCommand>::SharedPtr sub_hazard_lights_;
};

}  // namespace vehicle_cmd_gate2

#endif  // COMMAND__SUBSCRIPTION_HPP_
