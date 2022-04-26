// Copyright 2022 TIER IV, Inc.
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

#include "component_state_machine/machine.hpp"

#include <iostream>

enum class MyStates : uint16_t { NORMAL, ERROR, STATE3 };

enum class MyEvents : uint16_t { EVENT1, EVENT2 };

int main()
{
  using component_state_machine::Machine;
  Machine<MyStates, MyEvents> machine;

  machine.CreateState(MyStates::NORMAL);
  machine.CreateState(MyStates::ERROR);
  machine.SetTransition(MyEvents::EVENT1, MyStates::NORMAL, MyStates::ERROR);

  machine.SetInitialState(MyStates::NORMAL);
  machine.Initialize();
  machine.HandleEvent(MyEvents::EVENT1);

  auto state = machine.GetState();
  std::cout << static_cast<std::underlying_type_t<MyStates>>(state) << std::endl;
}
