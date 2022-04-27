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

#ifndef COMPONENT_STATE_MACHINE__STATE_MACHINE_HPP_
#define COMPONENT_STATE_MACHINE__STATE_MACHINE_HPP_

#include "component_state_machine/types.hpp"

#include <memory>
#include <string>
#include <unordered_map>

namespace component_state_machine
{

struct StateData
{
  explicit StateData(const StateID id) : id(id) {}
  const StateID id;
  std::unordered_map<EventID, StateID> transitions;

  // non-copyable
  StateData(const StateData &) = delete;
  StateData & operator=(const StateData &) = delete;
};

struct EventData
{
  explicit EventData(const EventID id) : id(id) {}
  const EventID id;

  // non-copyable
  EventData(const EventData &) = delete;
  EventData & operator=(const EventData &) = delete;
};

class StateMachine
{
public:
  static constexpr StateID kStateUnknown = StateID{0};
  static constexpr EventID kEventUnknown = EventID{0};

  StateMachine();
  void CreateState(StateID sid);
  void CreateEvent(EventID eid);
  void SetInitialState(StateID sid);
  void SetTransition(EventID eid, StateID src_sid, StateID dst_sid);

  StateID GetState() const;
  void Initialize();
  void HandleEvent(EventID eid);
  void Dump();

private:
  StateID initial_state_;
  StateID current_state_;
  std::unordered_map<StateID, StateData> states_;
  std::unordered_map<EventID, EventData> events_;
};

class StateMachineLoader
{
public:
  void BindState(uint16_t value, std::string name);
  void BindEvent(uint16_t value, std::string name);
  void LoadYAML(StateMachine & machine, std::string path);

private:
  std::unordered_map<std::string, StateID> states_;
  std::unordered_map<std::string, EventID> events_;
};

}  // namespace component_state_machine

#endif  // COMPONENT_STATE_MACHINE__STATE_MACHINE_HPP_
