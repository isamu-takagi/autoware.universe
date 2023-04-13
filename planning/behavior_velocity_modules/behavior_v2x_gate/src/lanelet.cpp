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

#include <rclcpp/logging.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>

namespace
{

using LaneletLayer = lanelet::LaneletLayer;
using ConstLines = std::vector<lanelet::ConstLineString3d>;
using LaneToLine = std::unordered_map<lanelet::Id, lanelet::ConstLineString3d>;
using V2xGate = lanelet::autoware::V2xGate;

template <class Point>
auto center_point(Point p, const Point & q)
{
  boost::geometry::add_point(p, q);
  boost::geometry::multiply_value(p, 0.5);
  return p;
}

LaneToLine create_lane_to_line(const LaneletLayer & lanes, const ConstLines & lines)
{
  LaneToLine mapping;
  for (const auto & line : lines) {
    const auto center = center_point(line[0].basicPoint(), line[1].basicPoint());
    for (const auto & lane : lanes) {
      const auto dist = lanelet::geometry::distance(lane.polygon3d(), center);
      if (dist < 1e-3) {
        mapping[lane.id()] = line;
      }
    }
  }
  return mapping;
}

}  // namespace

namespace behavior_velocity_planner::v2x_gate
{

GateArea::GateArea(const BaseType::ConstPtr gate, const lanelet::LaneletMapPtr map)
{
  base_ = gate;
  acquire_lines_ = create_lane_to_line(map->laneletLayer, gate->getAcquireLines());
  release_lines_ = create_lane_to_line(map->laneletLayer, gate->getReleaseLines());
}

GateAreas get_all_gate_areas(const lanelet::LaneletMapPtr map)
{
  GateAreas gates;
  for (const auto & element : map->regulatoryElementLayer) {
    const auto gate = std::dynamic_pointer_cast<const V2xGate>(element);
    if (gate) {
      gates.push_back(std::make_shared<GateArea>(gate, map));
    }
  }
  return gates;
}

LaneToGateAreas get_lane_to_gate_areas(const lanelet::LaneletMapPtr map)
{
  LaneToGateAreas mapping;
  const auto gates = get_all_gate_areas(map);
  for (const auto & gate : gates) {
    for (const auto & [lane_id, line] : gate->getAcquireLines()) {
      mapping[lane_id].push_back(gate);
    }
  }
  return mapping;
}

}  // namespace behavior_velocity_planner::v2x_gate
