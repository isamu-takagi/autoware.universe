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

#include <vector>

namespace vehicle_cmd_gate2
{

class CommandSelector : public CommandBridge
{
public:
  CommandSelector();
  void register_input(CommandSender * input);
  void select(size_t index);

private:
  std::vector<CommandSender *> inputs_;
  size_t input_index_;
};

}  // namespace vehicle_cmd_gate2

#endif  // COMMAND__SELECTOR_HPP_
