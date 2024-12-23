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

namespace autoware::control_cmd_gate
{

class CommandSubscription : public CommandInput
{
public:
  CommandSubscription(rclcpp::Node & node, const std::string & name);
  void resend_last_command() override;

private:
  void on_control(Control::ConstSharedPtr msg);
  void on_gear(GearCommand::ConstSharedPtr msg);
  void on_turn_indicators(TurnIndicatorsCommand::ConstSharedPtr msg);
  void on_hazard_lights(HazardLightsCommand::ConstSharedPtr msg);

  rclcpp::Subscription<Control>::SharedPtr sub_control_;
  rclcpp::Subscription<GearCommand>::SharedPtr sub_gear_;
  rclcpp::Subscription<TurnIndicatorsCommand>::SharedPtr sub_turn_indicators_;
  rclcpp::Subscription<HazardLightsCommand>::SharedPtr sub_hazard_lights_;

  GearCommand::ConstSharedPtr last_gear_;
  TurnIndicatorsCommand::ConstSharedPtr last_turn_indicators_;
  HazardLightsCommand::ConstSharedPtr last_hazard_lights_;
};

}  // namespace autoware::control_cmd_gate

#endif  // COMMAND__SUBSCRIPTION_HPP_
