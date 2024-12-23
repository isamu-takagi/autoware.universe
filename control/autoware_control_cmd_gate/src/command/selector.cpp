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

#include "selector.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::control_cmd_gate
{

CommandSource::CommandSource(CommandOutput * output) : CommandBridge(output)
{
}

void CommandSource::set_output(CommandOutput * output)
{
  CommandBridge::set_output(output);
  if (gear_) on_gear(gear_);
  if (turn_indicators_) on_turn_indicators(turn_indicators_);
  if (hazard_lights_) on_hazard_lights(hazard_lights_);
}

void CommandSource::on_control(const Control::ConstSharedPtr msg)
{
  // NOTE: Control is not saved because it is sent periodically.
  CommandBridge::on_control(msg);
}

void CommandSource::on_gear(const GearCommand::ConstSharedPtr msg)
{
  gear_ = msg;
  CommandBridge::on_gear(msg);
}

void CommandSource::on_turn_indicators(const TurnIndicatorsCommand::ConstSharedPtr msg)
{
  turn_indicators_ = msg;
  CommandBridge::on_turn_indicators(msg);
}

void CommandSource::on_hazard_lights(const HazardLightsCommand::ConstSharedPtr msg)
{
  hazard_lights_ = msg;
  CommandBridge::on_hazard_lights(msg);
}

CommandSelector::CommandSelector()
{
  ignore_ = std::make_unique<CommandIgnore>();
}

bool CommandSelector::add_source(const std::string & name, std::unique_ptr<CommandInput> && source)
{
  source->set_output(ignore_.get());
  return sources_.emplace(name, std::move(source)).second;
}

void CommandSelector::set_output(std::unique_ptr<CommandOutput> && output)
{
  output_ = output;
}

CommandOutput * CommandSelector::create(const std::string & name)
{
  return sources_.emplace(name, std::make_unique<CommandSource>(ignore_.get())).first->second.get();
}

bool CommandSelector::select(const std::string & name)
{
  const auto iter = sources_.find(name);
  if (iter == sources_.end()) {
    return false;
  }
  if (current_) {
    current_->set_output(ignore_.get());
  }
  current_ = iter->second.get();
  current_->set_output(output_.get());

  return true;
}

}  // namespace autoware::control_cmd_gate
