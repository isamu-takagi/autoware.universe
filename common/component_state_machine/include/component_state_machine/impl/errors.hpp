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

#ifndef COMPONENT_STATE_MACHINE__IMPL__ERRORS_HPP_
#define COMPONENT_STATE_MACHINE__IMPL__ERRORS_HPP_

#include <stdexcept>

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

}  // namespace component_state_machine

#endif  // COMPONENT_STATE_MACHINE__IMPL__ERRORS_HPP_
