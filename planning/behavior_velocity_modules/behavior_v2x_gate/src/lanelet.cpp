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

template <typename Point>
auto center_point(Point p1, Point p2)
{
  boost::geometry::add_point(p1, p2);
  boost::geometry::multiply_value(p1, 0.5);
  return p1;
}

}  // namespace

namespace behavior_v2x_gate
{

std::vector<V2xGate::ConstPtr> get_all_v2x_gates(const lanelet::LaneletMapPtr map)
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

void create_v2x_gate_map(const lanelet::LaneletMapPtr map)
{
  const auto logger = rclcpp::get_logger("test_v2x_gate");

  const auto gates = get_all_v2x_gates(map);
  for (const auto & gate : gates) {
    RCLCPP_INFO_STREAM(logger, "===== " << gate->id() << " =====");
    for (const auto & line : gate->getAcquireLines()) {
      RCLCPP_INFO_STREAM(logger, "line " << line.id());

      const auto center = center_point(line[0].basicPoint(), line[1].basicPoint());
      RCLCPP_INFO_STREAM(
        logger, "  - center: " << center.x() << " " << center.y() << " " << center.z());

      for (const auto & lane : map->laneletLayer) {
        const auto dist = lanelet::geometry::distance(lane.polygon3d().basicPolygon(), center);
        RCLCPP_INFO_STREAM(logger, "    - lane " << lane.id() << ": " << dist);
      }
    }
  }
}

}  // namespace behavior_v2x_gate
