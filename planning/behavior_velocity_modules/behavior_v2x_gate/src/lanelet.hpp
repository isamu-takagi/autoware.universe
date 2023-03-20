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

#ifndef LANELET_HPP_
#define LANELET_HPP_

#include <lanelet2_extension/regulatory_elements/v2x_gate.hpp>

#include <memory>
#include <vector>

namespace behavior_v2x_gate
{

using lanelet::autoware::V2xGate;
std::vector<V2xGate::ConstPtr> get_all_v2x_gates(lanelet::LaneletMapPtr map);

}  // namespace behavior_v2x_gate

#endif  // LANELET_HPP_
