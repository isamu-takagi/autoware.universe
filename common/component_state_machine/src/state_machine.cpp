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

#include <yaml-cpp/yaml.h>

namespace component_state_machine
{

struct ErrorUnknownID : public std::runtime_error
{
  ErrorUnknownID() : std::runtime_error("unknown id") {}
};

struct ErrorExistID : public std::runtime_error
{
  ErrorExistID() : std::runtime_error("exist id") {}
};

template <class T, class U>
void check_empty(T & map, const U & id)
{
  if (id == U{0}) {
    throw ErrorUnknownID();
  }
  if (map.count(id)) {
    throw ErrorExistID();
  }
}

template <class T, class U>
void check_exist(T & map, const U & id)
{
  if (id == U{0}) {
    throw ErrorUnknownID();
  }
  if (!map.count(id)) {
    throw ErrorExistID();
  }
}

StateMachine::StateMachine()
{
  current_state_ = kStateUnknown;
  initial_state_ = kStateUnknown;
}

void StateMachine::CreateState(StateID sid)
{
  check_empty(states_, sid);
  states_.emplace(sid, sid);
}

void StateMachine::CreateEvent(EventID eid)
{
  check_empty(events_, eid);
  events_.emplace(eid, eid);
}

void StateMachine::SetInitialState(StateID sid)
{
  check_exist(states_, sid);
  initial_state_ = sid;
}

void StateMachine::SetTransition(EventID eid, StateID src_sid, StateID dst_sid)
{
  check_exist(events_, eid);
  check_exist(states_, src_sid);
  check_exist(states_, dst_sid);
  states_.at(src_sid).transitions.emplace(eid, dst_sid);
}

StateID StateMachine::GetState() const { return current_state_; }

void StateMachine::Initialize() { current_state_ = initial_state_; }

void StateMachine::HandleEvent(EventID eid)
{
  const auto & transitions = states_.at(current_state_).transitions;
  if (transitions.count(eid)) {
    current_state_ = transitions.at(eid);
  }
}

void StateMachineLoader::BindState(uint16_t value, std::string name)
{
  states_[name] = StateID{value};
}

void StateMachineLoader::BindEvent(uint16_t value, std::string name)
{
  events_[name] = EventID{value};
}

void StateMachineLoader::LoadYAML(StateMachine & machine, std::string path)
{
  YAML::Node yaml = YAML::LoadFile(path);
  for (const auto & state : yaml["states"]) {
    const auto name = state.as<std::string>();
    machine.CreateState(states_.at(name));
  }
  for (const auto & event : yaml["events"]) {
    const auto name = event.as<std::string>();
    machine.CreateEvent(events_.at(name));
  }
  for (const auto & state : yaml["initial"]) {
    const auto name = state.as<std::string>();
    machine.SetInitialState(states_.at(name));
  }
  for (const auto & transition : yaml["transitions"]) {
    const auto event = events_.at(transition["event"].as<std::string>());
    const auto src = states_.at(transition["src"].as<std::string>());
    const auto dst = states_.at(transition["dst"].as<std::string>());
    machine.SetTransition(event, src, dst);
  }
}

}  // namespace component_state_machine

#include <iostream>

namespace component_state_machine
{

void StateMachine::Dump()
{
  std::cout << "states:";
  for (const auto & pair : states_) {
    std::cout << " " << pair.first.value;
  }
  std::cout << std::endl;

  std::cout << "events:";
  for (const auto & pair : events_) {
    std::cout << " " << pair.first.value;
  }
  std::cout << std::endl;

  std::cout << "transitions:" << std::endl;
  for (const auto & pair : states_) {
    for (const auto & transit : pair.second.transitions) {
      std::cout << " - " << transit.first.value << " (" << pair.first.value << "=>"
                << transit.second.value << ")" << std::endl;
    }
  }
}

}  // namespace component_state_machine
