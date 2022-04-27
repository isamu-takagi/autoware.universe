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

#ifndef COMPONENT_STATE_MACHINE__TYPES_HPP_
#define COMPONENT_STATE_MACHINE__TYPES_HPP_

#include <cstdint>
#include <functional>

namespace component_state_machine
{

struct StateID
{
  uint16_t value;
};

struct EventID
{
  uint16_t value;
};

inline bool operator==(const StateID & l, const StateID & r)
{
  return l.value == r.value;
}

inline bool operator==(const EventID & l, const EventID & r)
{
  return l.value == r.value;
}

}  // namespace component_state_machine

namespace std
{

template <>
struct hash<component_state_machine::StateID>
{
  size_t operator()(const component_state_machine::StateID & id) const
  {
    return hash<uint16_t>()(id.value);
  }
};

template <>
struct hash<component_state_machine::EventID>
{
  size_t operator()(const component_state_machine::EventID & id) const
  {
    return hash<uint16_t>()(id.value);
  }
};

}  // namespace std

#endif  // COMPONENT_STATE_MACHINE__TYPES_HPP_
