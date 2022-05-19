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

#include "component_state_machine/impl/errors.hpp"

#include <memory>
#include <string>
#include <unordered_map>

namespace component_state_machine
{

template <class StateID, class EventID>
struct BaseStateData
{
  explicit BaseStateData(const StateID id) : id(id) {}
  const StateID id;
  std::unordered_map<EventID, StateID> transitions;

  // non-copyable
  BaseStateData(const BaseStateData &) = delete;
  BaseStateData & operator=(const BaseStateData &) = delete;
};

template <class StateID, class EventID>
struct BaseEventData
{
  explicit BaseEventData(const EventID id) : id(id) {}
  const EventID id;

  // non-copyable
  BaseEventData(const BaseEventData &) = delete;
  BaseEventData & operator=(const BaseEventData &) = delete;
};

template <class StateID, class EventID>
class StateMachine
{
public:
  using StateData = BaseStateData<StateID, EventID>;
  using EventData = BaseEventData<StateID, EventID>;

  explicit StateMachine(StateID initial_state)
  {
    current_state_ = initial_state;
    initial_state_ = initial_state;
  }

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

template <class StateID, class EventID>
class StateMachineLoader
{
public:
  void BindState(StateID value, std::string name);
  void BindEvent(EventID value, std::string name);
  void LoadYAML(StateMachine<StateID, EventID> & machine, std::string path);

private:
  std::unordered_map<std::string, StateID> states_;
  std::unordered_map<std::string, EventID> events_;
};

}  // namespace component_state_machine

#endif  // COMPONENT_STATE_MACHINE__STATE_MACHINE_HPP_
