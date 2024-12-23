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
  void on_control(Control::ConstSharedPtr) override {}
  void on_gear(GearCommand::ConstSharedPtr) override {}
  void on_turn_indicators(TurnIndicatorsCommand::ConstSharedPtr) override {}
  void on_hazard_lights(HazardLightsCommand::ConstSharedPtr) override {}
};

class CommandSelector
{
public:
  CommandSelector();
  void add_source(const std::string & name, std::unique_ptr<CommandInput> && source);
  void set_output(std::unique_ptr<CommandOutput> && output);
  bool select(const std::string & name);

private:
  std::unordered_map<std::string, std::unique_ptr<CommandInput>> sources_;
  std::unique_ptr<CommandIgnore> ignore_;
  std::unique_ptr<CommandOutput> output_;
};

}  // namespace autoware::control_cmd_gate

#endif  // COMMAND__SELECTOR_HPP_
