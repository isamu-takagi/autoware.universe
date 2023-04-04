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

#include <behavior_velocity_planner/planner_data/common.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utilization/arc_lane_util.hpp>  // TODO(Takagi, Isamu): move header
#include <utilization/util.hpp>

#include <utility>

namespace behavior_velocity_planner::v2x_gate
{

std::optional<arc_lane_utils::PathIndexWithPose> get_first_cross_point(
  const PathWithLaneId & path, const FrameData & frame,
  const std::unordered_map<lanelet::Id, lanelet::ConstLineString3d> & lines)
{
  // TODO(Takagi, Isamu): Use stop margin
  const double stop_margin = 0.0;
  const double stop_offset = frame.data->common->vehicle_info.max_longitudinal_offset_m;
  const double line_extend = frame.data->common->stop_line_extend_length;

  // TODO(Takagi, Isamu): Select closest point.
  for (const auto & [lane_id, line] : lines) {
    const auto stop_line = planning_utils::extendLine(line[0], line[1], line_extend);
    const auto stop_pair =
      arc_lane_utils::createTargetPoint(path, stop_line, lane_id, stop_margin, stop_offset);
    if (stop_pair) {
      const auto & [stop_index, stop_pose] = stop_pair.value();
      const auto stop_seg_index =
        planning_utils::calcSegmentIndexFromPointIndex(path.points, stop_pose.position, stop_index);
      return std::make_pair(stop_seg_index, stop_pose);
    }
  }
  return std::nullopt;
}

SceneModule::SceneModule(const V2xGateData::ConstPtr & data)
{
  data_ = data;
}

void SceneModule::plan(PathWithLaneId * path, const FrameData & frame)
{
  using arc_lane_utils::PathIndexWithPose;

  (void)path;
  (void)frame;

  const auto logger = rclcpp::get_logger("behavior_velocity_planner.v2x_gate");
  RCLCPP_INFO_STREAM(logger, "scene module: " << data_->gate->id());

  const auto acquire_point = get_first_cross_point(*path, frame, data_->acquire_lines);
  if (acquire_point) {
    const auto [index, pose] = acquire_point.value();
    RCLCPP_INFO_STREAM(logger, " - acquire line: " << index);
    planning_utils::insertStopPoint(pose.position, index, *path);
  }

  const auto release_point = get_first_cross_point(*path, frame, data_->release_lines);
  if (release_point) {
    const auto [index, pose] = release_point.value();
    RCLCPP_INFO_STREAM(logger, " - release line: " << index);
  }
}

}  // namespace behavior_velocity_planner::v2x_gate
