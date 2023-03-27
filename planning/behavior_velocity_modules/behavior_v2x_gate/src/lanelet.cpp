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

template <class Point>
auto center_point(Point p, const Point & q)
{
  boost::geometry::add_point(p, q);
  boost::geometry::multiply_value(p, 0.5);
  return p;
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

std::vector<V2xGateData::ConstPtr> create_v2x_gate_data(const lanelet::LaneletMapPtr map)
{
  using LineStringList = std::vector<lanelet::ConstLineString3d>;
  const auto bind_lanelet = [map](V2xGateData & data, const LineStringList & lines) {
    for (const auto & line : lines) {
      const auto center = center_point(line[0].basicPoint(), line[1].basicPoint());
      for (const auto & lane : map->laneletLayer) {
        const auto dist = lanelet::geometry::distance(lane.polygon3d(), center);
        if (dist < 1e-3) {
          data.lines[lane.id()].push_back(line);
        }
      }
    }
  };

  std::vector<V2xGateData::ConstPtr> list;
  for (const auto & gate : get_all_v2x_gates(map)) {
    const auto data = std::make_shared<V2xGateData>();
    data->gate = gate;
    bind_lanelet(*data, gate->getAcquireLines());
    bind_lanelet(*data, gate->getReleaseLines());
    list.push_back(data);
  }
  return list;
}

V2xGateMap create_lanelet_to_v2x_gate(const lanelet::LaneletMapPtr map)
{
  V2xGateMap mapping;
  const auto gates = create_v2x_gate_data(map);
  for (const auto & gate : gates) {
    for (const auto & [lane_id, line] : gate->lines) {
      mapping[lane_id].push_back(gate);
    }
  }
  return mapping;
}

}  // namespace behavior_v2x_gate
