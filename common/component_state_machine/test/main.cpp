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

#include "component_state_machine/state_machine.hpp"

#include <iostream>

using namespace component_state_machine;

void print_state(const StateMachine & machine)
{
  auto state = machine.GetState();
  std::cout << state.value << std::endl;
}

int main()
{
  StateMachine machine;
  machine.CreateState(StateID{1});
  machine.CreateState(StateID{2});
  machine.SetTransition(EventID{1}, StateID{1}, StateID{2});
  machine.SetInitialState(StateID{1});

  machine.Initialize();
  print_state(machine);

  machine.HandleEvent(EventID{1});
  print_state(machine);
}
