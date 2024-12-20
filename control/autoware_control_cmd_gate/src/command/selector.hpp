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

#ifndef COMMAND__SELECTOR_HPP_
#define COMMAND__SELECTOR_HPP_

#include "interface.hpp"

#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::control_cmd_gate
{

class CommandIgnore : public CommandOutput
{
public:
  void on_control(const Control::ConstSharedPtr) override {}
  void on_gear(const GearCommand::ConstSharedPtr) override {}
  void on_turn_indicators(const TurnIndicatorsCommand::ConstSharedPtr) override {}
  void on_hazard_lights(const HazardLightsCommand::ConstSharedPtr) override {}
};

class CommandSource : public CommandBridge
{
public:
  explicit CommandSource(CommandOutput * output);
  void set_output(CommandOutput * output);

protected:
  void on_control(const Control::ConstSharedPtr msg);
  void on_gear(const GearCommand::ConstSharedPtr msg);
  void on_turn_indicators(const TurnIndicatorsCommand::ConstSharedPtr msg);
  void on_hazard_lights(const HazardLightsCommand::ConstSharedPtr msg);

private:
  GearCommand::ConstSharedPtr gear_;
  TurnIndicatorsCommand::ConstSharedPtr turn_indicators_;
  HazardLightsCommand::ConstSharedPtr hazard_lights_;
};

class CommandSelector
{
public:
  CommandSelector();
  CommandOutput * create(const std::string & name);
  bool select(const std::string & name);

private:
  std::unordered_map<std::string, std::unique_ptr<CommandSource>> sources_;
  std::unique_ptr<CommandIgnore> ignore_;
  std::unique_ptr<CommandOutput> output_;
  CommandSource * current_;
};

}  // namespace autoware::control_cmd_gate

#endif  // COMMAND__SELECTOR_HPP_
