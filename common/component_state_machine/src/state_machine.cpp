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

namespace component_state_machine
{

struct StateExistError : public std::runtime_error
{
  StateExistError() : std::runtime_error("state exists") {}
};

StateMachine::StateMachine()
{
  current_state_ = kStateUnknown;
  initial_state_ = kStateUnknown;
}

void StateMachine::CreateState(StateID sid)
{
  if (sid == kStateUnknown) {
    throw StateExistError();
  }
  states_.emplace(sid, sid);
}

void StateMachine::CreateEvent(EventID eid)
{
  if (eid == kEventUnknown) {
    throw StateExistError();
  }
  events_.emplace(eid, eid);
}

void StateMachine::SetInitialState(StateID sid)
{
  initial_state_ = sid;
}

void StateMachine::SetTransition(EventID eid, StateID src_sid, StateID dst_sid)
{
  states_.at(src_sid).transitions.emplace(eid, dst_sid);
}

StateID StateMachine::GetState() const
{
  return current_state_;
}

void StateMachine::Initialize()
{
  current_state_ = initial_state_;
}

void StateMachine::HandleEvent(EventID eid)
{
  const auto & transitions = states_.at(current_state_).transitions;
  if (transitions.count(eid)) {
    current_state_ = transitions.at(eid);
  }
}

}  // namespace component_state_machine
