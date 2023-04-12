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

#include "module.hpp"

#include <lanelet2_extension/regulatory_elements/v2x_gate.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace behavior_velocity_planner::v2x_gate
{

class GateArea
{
public:
  using BaseTypePtr = lanelet::autoware::V2xGate::ConstPtr;
  GateArea(const BaseTypePtr gate, const lanelet::LaneletMapPtr map);

  lanelet::Id id() { return base_->id(); }
  std::string getCategory() { return base_->getCategory(); }

private:
  BaseTypePtr base_;
};

using V2xGateDataSet = std::unordered_set<V2xGateData::ConstPtr>;
using LaneToGate = std::unordered_map<lanelet::Id, std::vector<V2xGateData::ConstPtr>>;
using lanelet::autoware::V2xGate;

std::vector<V2xGate::ConstPtr> get_all_v2x_gates(const lanelet::LaneletMapPtr map);
std::vector<V2xGateData::ConstPtr> create_v2x_gate_data(const lanelet::LaneletMapPtr map);
LaneToGate create_lanelet_to_v2x_gate(const lanelet::LaneletMapPtr map);

}  // namespace behavior_velocity_planner::v2x_gate

#endif  // LANELET_HPP_
