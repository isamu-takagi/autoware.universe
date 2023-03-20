// Copyright 2023 The Autoware Contributors
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

#include "lanelet.hpp"

#include <lanelet2_core/LaneletMap.h>

namespace behavior_v2x_gate
{

std::vector<V2xGate::ConstPtr> get_all_v2x_gates(lanelet::LaneletMapPtr map)
{
  std::vector<V2xGate::ConstPtr> gates;
  for (const auto & element : map->regulatoryElementLayer) {
    const auto gate = std::dynamic_pointer_cast<const V2xGate>(element);
    if (gate) {
      gates.push_back(gate);
    }
  }
  return gates;
}

}  // namespace behavior_v2x_gate
