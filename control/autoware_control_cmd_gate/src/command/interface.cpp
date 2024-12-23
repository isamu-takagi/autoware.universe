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

namespace autoware::control_cmd_gate
{

CommandInput::CommandInput()
{
  output_ = nullptr;
}

CommandInput::CommandInput(CommandOutput * output)
{
  set_output(output);
}

void CommandInput::set_output(CommandOutput * output)
{
  if (!output) {
    throw std::logic_error("command output is nullptr");
  }
  output_ = output;
}

void CommandInput::send_control(const Control::ConstSharedPtr msg)
{
  output_->on_control(msg);
}

void CommandInput::send_gear(const GearCommand::ConstSharedPtr msg)
{
  output_->on_gear(msg);
}

void CommandInput::send_turn_indicators(const TurnIndicatorsCommand::ConstSharedPtr msg)
{
  output_->on_turn_indicators(msg);
}

void CommandInput::send_hazard_lights(const HazardLightsCommand::ConstSharedPtr msg)
{
  return output_->on_hazard_lights(msg);
}

void CommandBridge::on_control(const Control::ConstSharedPtr msg)
{
  return send_control(msg);
}

void CommandBridge::on_gear(const GearCommand::ConstSharedPtr msg)
{
  return send_gear(msg);
}

void CommandBridge::on_turn_indicators(const TurnIndicatorsCommand::ConstSharedPtr msg)
{
  return send_turn_indicators(msg);
}

void CommandBridge::on_hazard_lights(const HazardLightsCommand::ConstSharedPtr msg)
{
  return send_hazard_lights(msg);
}

/*
void CommandSource::select(std::unique_ptr<CommandOutput> && output)
{
  output_ = std::move(output);
}

void CommandSource::select(CommandSource & other)
{
  if (!other.output_) {
    throw std::logic_error("command source with no output is specified");
  }

  output_ = std::move(other.output_);
  if (gear_) on_gear(gear_);
  if (turn_indicators_) on_turn_indicators(turn_indicators_);
  if (hazard_lights_) on_hazard_lights(hazard_lights_);
}

void CommandSource::on_control(const Control::ConstSharedPtr msg)
{
  // NOTE: Control is not saved because it is sent periodically.
  if (output_) output_->on_control(msg);
}

void CommandSource::on_gear(const GearCommand::ConstSharedPtr msg)
{
  gear_ = msg;
  if (output_) output_->on_gear(msg);
}

void CommandSource::on_turn_indicators(const TurnIndicatorsCommand::ConstSharedPtr msg)
{
  turn_indicators_ = msg;
  if (output_) output_->on_turn_indicators(msg);
}

void CommandSource::on_hazard_lights(const HazardLightsCommand::ConstSharedPtr msg)
{
  hazard_lights_ = msg;
  if (output_) output_->on_hazard_lights(msg);
}
*/

}  // namespace autoware::control_cmd_gate
