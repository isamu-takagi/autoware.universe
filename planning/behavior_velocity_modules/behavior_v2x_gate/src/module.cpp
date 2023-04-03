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

#include "module.hpp"

#include <rclcpp/rclcpp.hpp>
#include <utilization/arc_lane_util.hpp>  // TODO(Takagi, Isamu): move header
#include <utilization/util.hpp>

#include <utility>

namespace behavior_velocity_planner::v2x_gate
{

SceneModule::SceneModule(const V2xGateData::ConstPtr data) { data_ = std::move(data); }

void SceneModule::plan(PathWithLaneId * path, const FrameData & frame)
{
  (void)path;
  (void)frame;

  const auto logger = rclcpp::get_logger("behavior_velocity_planner.v2x_gate");
  RCLCPP_INFO_STREAM(logger, "scene module: " << data_->gate->id());

  // TODO(Takagi, Isamu): set stop params
  const double stop_margin = 0.0;
  const double stop_offset = 0.0;  // planner_data_->vehicle_info_.max_longitudinal_offset_m;
  const double line_extend = 0.0;

  for (const auto & [lane_id, line] : data_->acquire_lines) {
    const auto stop_line = planning_utils::extendLine(line[0], line[1], line_extend);
    const auto stop_pose =
      arc_lane_utils::createTargetPoint(*path, stop_line, lane_id, stop_margin, stop_offset);
    if (stop_pose) {
      RCLCPP_INFO_STREAM(logger, " - stop pose: " << stop_pose->first);

      const auto stop_seg_idx = planning_utils::calcSegmentIndexFromPointIndex(
        path->points, stop_pose->second.position, stop_pose->first);
      planning_utils::insertStopPoint(stop_pose->second.position, stop_seg_idx, *path);
    }
  }
}

}  // namespace behavior_velocity_planner::v2x_gate
