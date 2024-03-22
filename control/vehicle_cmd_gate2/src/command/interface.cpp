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

#include "interface.hpp"

namespace vehicle_cmd_gate2
{

void CommandSender::connect(CommandReceiver * receiver)
{
  receiver_ = receiver;
}

void CommandSender::send_control(const AckermannControlCommand::ConstSharedPtr msg)
{
  if (receiver_) receiver_->on_control(msg);
}

void CommandSender::send_gear(const GearCommand::ConstSharedPtr msg)
{
  if (receiver_) receiver_->on_gear(msg);
}

void CommandSender::send_turn_indicators(const TurnIndicatorsCommand::ConstSharedPtr msg)
{
  if (receiver_) receiver_->on_turn_indicators(msg);
}

void CommandSender::send_hazard_lights(const HazardLightsCommand::ConstSharedPtr msg)
{
  if (receiver_) receiver_->on_hazard_lights(msg);
}

void CommandBridge::on_control(const AckermannControlCommand::ConstSharedPtr msg)
{
  send_control(msg);
}

void CommandBridge::on_gear(const GearCommand::ConstSharedPtr msg)
{
  send_gear(msg);
}

void CommandBridge::on_turn_indicators(const TurnIndicatorsCommand::ConstSharedPtr msg)
{
  send_turn_indicators(msg);
}

void CommandBridge::on_hazard_lights(const HazardLightsCommand::ConstSharedPtr msg)
{
  send_hazard_lights(msg);
}

}  // namespace vehicle_cmd_gate2
