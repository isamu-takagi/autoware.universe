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
#include <string>
#include <unordered_map>
#include <vector>

namespace behavior_velocity_planner::v2x_gate
{

class GateArea
{
public:
  using ConstSharedPtr = std::shared_ptr<const GateArea>;
  using BaseType = lanelet::autoware::V2xGate;
  GateArea(const BaseType::ConstPtr gate, const lanelet::LaneletMapPtr map);

  lanelet::Id id() const { return base_->id(); }
  std::string getCategory() const { return base_->getCategory(); }

  auto getAcquireLines() const { return acquire_lines_; }
  auto getReleaseLines() const { return release_lines_; }

private:
  BaseType::ConstPtr base_;
  std::unordered_map<lanelet::Id, lanelet::ConstLineString3d> acquire_lines_;
  std::unordered_map<lanelet::Id, lanelet::ConstLineString3d> release_lines_;
};

using GateAreas = std::vector<GateArea::ConstSharedPtr>;
using LaneToGateAreas = std::unordered_map<lanelet::Id, GateAreas>;
LaneToGateAreas get_lane_to_gate_areas(const lanelet::LaneletMapPtr map);

}  // namespace behavior_velocity_planner::v2x_gate

#endif  // LANELET_HPP_
