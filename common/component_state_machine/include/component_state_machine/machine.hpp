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

#ifndef COMPONENT_STATE_MACHINE__MACHINE_HPP_
#define COMPONENT_STATE_MACHINE__MACHINE_HPP_

#include <memory>
#include <optional>
#include <unordered_map>

namespace component_state_machine
{

class Adaptor
{
  // CreateMachine()
};

template <class StateID, class EventID>
struct StateData
{
  explicit StateData(const StateID id) : id(id) {}
  const StateID id;
  std::unordered_map<EventID, StateID> transitions;

  // non-copyable
  StateData(const StateData &) = delete;
  StateData & operator=(const StateData &) = delete;
};

template <class StateID, class EventID>
struct EventData
{
  explicit EventData(const EventID id) : id(id) {}
  const EventID id;

  // non-copyable
  EventData(const EventData &) = delete;
  EventData & operator=(const EventData &) = delete;
};

template <class StateID, class EventID>
class Machine
{
public:
  using State = StateData<StateID, EventID>;
  using Event = EventData<StateID, EventID>;

  StateID GetState() { return current_state_.value(); }

  void CreateState(StateID id) { states_.emplace(id, id); }

  void CreateEvent(EventID id) { events_.emplace(id, id); }

  void SetInitialState(StateID id) { initial_state_ = id; }

  void SetTransition(EventID event, StateID src, StateID dst)
  {
    states_.at(src).transitions.emplace(event, dst);
  }

  void Initialize() { current_state_ = initial_state_; }

  void HandleEvent(EventID event)
  {
    const auto & transitions = states_.at(current_state_.value()).transitions;
    if (transitions.count(event)) {
      current_state_ = transitions.at(event);
    }
  }

private:
  std::optional<StateID> initial_state_;
  std::optional<StateID> current_state_;
  std::unordered_map<StateID, State> states_;
  std::unordered_map<EventID, Event> events_;
};

}  // namespace component_state_machine

#endif  // COMPONENT_STATE_MACHINE__MACHINE_HPP_
