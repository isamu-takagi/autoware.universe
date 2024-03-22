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

#ifndef COMMAND__PUBLISHER_HPP_
#define COMMAND__PUBLISHER_HPP_

#include "interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace vehicle_cmd_gate2
{

class CommandPublisher : public CommandReceiver
{
public:
  CommandPublisher(rclcpp::Node & node, const std::string & ns);
  void on_control(const AckermannControlCommand::ConstSharedPtr msg) override;
  void on_gear(const GearCommand::ConstSharedPtr msg) override;
  void on_turn_indicators(const TurnIndicatorsCommand::ConstSharedPtr msg) override;
  void on_hazard_lights(const HazardLightsCommand::ConstSharedPtr msg) override;

private:
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_control_;
  rclcpp::Publisher<GearCommand>::SharedPtr pub_gear_;
  rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr pub_turn_indicators_;
  rclcpp::Publisher<HazardLightsCommand>::SharedPtr pub_hazard_lights_;
};

}  // namespace vehicle_cmd_gate2

#endif  // COMMAND__PUBLISHER_HPP_
