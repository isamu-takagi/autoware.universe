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

#include <utility>

namespace autoware::control_cmd_gate
{

void CommandInput::set_output(CommandOutput * output)
{
  if (!output) {
    throw std::logic_error("command output is nullptr");
  }
  output_ = output;
}

void CommandInput::add_filter(std::unique_ptr<CommandFilter> && filter)
{
  if (!filter) {
    throw std::logic_error("command filter is nullptr");
  }
  filters_.push_back(std::move(filter));
}

void CommandInput::send_control(Control::ConstSharedPtr msg)
{
  for (const auto & filter : filters_) {
    msg = filter->on_control(msg);
  }
  output_->on_control(msg);
}

void CommandInput::send_gear(GearCommand::ConstSharedPtr msg)
{
  for (const auto & filter : filters_) {
    msg = filter->on_gear(msg);
  }
  output_->on_gear(msg);
}

void CommandInput::send_turn_indicators(TurnIndicatorsCommand::ConstSharedPtr msg)
{
  for (const auto & filter : filters_) {
    msg = filter->on_turn_indicators(msg);
  }
  output_->on_turn_indicators(msg);
}

void CommandInput::send_hazard_lights(HazardLightsCommand::ConstSharedPtr msg)
{
  for (const auto & filter : filters_) {
    msg = filter->on_hazard_lights(msg);
  }
  output_->on_hazard_lights(msg);
}

Control::ConstSharedPtr CommandFilter::on_control(Control::ConstSharedPtr msg)
{
  return msg;
}

GearCommand::ConstSharedPtr CommandFilter::on_gear(GearCommand::ConstSharedPtr msg)
{
  return msg;
}

TurnIndicatorsCommand::ConstSharedPtr CommandFilter::on_turn_indicators(
  TurnIndicatorsCommand::ConstSharedPtr msg)
{
  return msg;
}

HazardLightsCommand::ConstSharedPtr CommandFilter::on_hazard_lights(
  HazardLightsCommand::ConstSharedPtr msg)
{
  return msg;
}

}  // namespace autoware::control_cmd_gate
