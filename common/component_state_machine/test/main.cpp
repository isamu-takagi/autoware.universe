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

using component_state_machine::EventID;
using component_state_machine::StateMachine;
using component_state_machine::StateMachineLoader;

namespace Message
{
constexpr uint16_t PREPARE = 1;
constexpr uint16_t READY = 2;
constexpr uint16_t DRIVING = 3;
}  // namespace Message

namespace Event
{
constexpr auto ENGAGE = EventID{1};
constexpr auto DISENGAGE = EventID{2};
constexpr auto READY = EventID{3};
constexpr auto UNREADY = EventID{4};
}  // namespace Event

void print_state(const StateMachine & machine)
{
  auto state = machine.GetState();
  std::cout << state.value << std::endl;
}

int main()
{
  StateMachine machine;
  {
    StateMachineLoader loader;
    loader.BindState(Message::PREPARE, "preparing");
    loader.BindState(Message::READY, "ready");
    loader.BindState(Message::DRIVING, "driving");
    loader.BindEvent(Event::ENGAGE.value, "engage");
    loader.BindEvent(Event::DISENGAGE.value, "disengage");
    loader.BindEvent(Event::READY.value, "ready");
    loader.BindEvent(Event::UNREADY.value, "unready");
    loader.LoadYAML(machine, "");
  }
  machine.Dump();

  machine.Initialize();
  print_state(machine);

  machine.HandleEvent(Event::READY);
  print_state(machine);

  machine.HandleEvent(Event::ENGAGE);
  print_state(machine);

  machine.HandleEvent(Event::DISENGAGE);
  print_state(machine);
}
