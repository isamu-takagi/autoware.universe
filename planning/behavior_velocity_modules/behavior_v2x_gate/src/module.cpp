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

#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_velocity_planner::v2x_gate
{

struct PathPoint
{
  geometry_msgs::msg::Pose pose;
  size_t index;
};

struct StopPoint : public PathPoint
{
  lanelet::ConstLineString3d line;
  double distance;
};

std::optional<StopPoint> get_first_cross_point(
  const PathWithLaneId & path, const FrameData & frame, const PathPoint & ego,
  const std::unordered_map<lanelet::Id, lanelet::ConstLineString3d> & lines)
{
  // TODO(Takagi, Isamu): Use stop margin
  const double stop_margin = 0.0;
  const double stop_offset = frame.data->vehicle_info.max_longitudinal_offset_m;
  const double line_extend = frame.data->stop_line_extend_length;

  std::optional<StopPoint> result;
  for (const auto & [lane_id, line] : lines) {
    const auto stop_line = planning_utils::extendLine(line[0], line[1], line_extend);
    const auto stop_pair =
      arc_lane_utils::createTargetPoint(path, stop_line, lane_id, stop_margin, stop_offset);
    if (stop_pair) {
      const auto & [stop_index, stop_pose] = stop_pair.value();
      const auto stop_seg_index =
        planning_utils::calcSegmentIndexFromPointIndex(path.points, stop_pose.position, stop_index);
      const auto stop_distance = motion_utils::calcSignedArcLength(
        path.points, ego.pose.position, ego.index, stop_pose.position, stop_seg_index);

      if (!result || stop_distance < result.value().distance) {
        result = StopPoint{stop_pose, stop_seg_index, line, stop_distance};
      }
    }
  }
  return result;
}

// TODO(Takagi, Isamu): Use scene manager interface.
template <class T>
PathPoint find_ego_segment_index(
  const std::vector<T> & points, const PlannerData2::ConstSharedPtr & p)
{
  const auto index = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    points, p->current_odometry->pose, p->ego_nearest_dist_threshold, p->ego_nearest_yaw_threshold);
  return PathPoint{p->current_odometry->pose, index};
}

using LaneToLines = std::unordered_map<lanelet::Id, lanelet::ConstLineString3d>;

LaneToLines filter_lines(const LaneToLines & lines, const std::set<lanelet::Id> & lanes)
{
  LaneToLines result;
  for (const auto & [key, value] : lines) {
    if (lanes.count(key)) {
      result[key] = value;
    }
  }
  return result;
}

std::set<lanelet::Id> get_keys(const LaneToLines & lines)
{
  std::set<lanelet::Id> keys;
  for (const auto & [key, value] : lines) {
    keys.insert(key);
  }
  return keys;
}

std::set<lanelet::Id> get_union(const std::set<lanelet::Id> & a, const std::set<lanelet::Id> & b)
{
  std::set<lanelet::Id> result;
  for (const auto & v : a) result.insert(v);
  for (const auto & v : b) result.insert(v);
  return result;
}

SceneModule::SceneModule(const GateArea::ConstSharedPtr & gate)
{
  gate_ = gate;
}

void SceneModule::plan(PathWithLaneId * path, const FrameData & frame)
{
  using arc_lane_utils::PathIndexWithPose;

  (void)path;
  (void)frame;

  const auto logger = rclcpp::get_logger("behavior_velocity_planner.v2x_gate");
  RCLCPP_INFO_STREAM(logger, "scene module: " << gate_->id());

  // Get gate lines on the path.
  const auto acquire_lines = filter_lines(gate_->getAcquireLines(), frame.lane_ids_on_path);
  const auto release_lines = filter_lines(gate_->getReleaseLines(), frame.lane_ids_on_path);

  ClientStatus status;
  status.gates = get_union(get_keys(acquire_lines), get_keys(release_lines));

  const auto ego = find_ego_segment_index(path->points, frame.data);
  RCLCPP_INFO_STREAM(logger, " - current pose: " << ego.index);

  const auto acquire_point = get_first_cross_point(*path, frame, ego, acquire_lines);
  const auto release_point = get_first_cross_point(*path, frame, ego, release_lines);

  if (acquire_point && release_point) {
    const auto acquire_id = acquire_point.value().line.id();
    const auto release_id = release_point.value().line.id();
    RCLCPP_INFO_STREAM(logger, " - target gates: " << acquire_id << " " << release_id);
  }

  /*
    const auto [dist, index, pose] = acquire_point.value();
    RCLCPP_INFO_STREAM(logger, " - acquire line: " << index << " " << dist);
    planning_utils::insertStopPoint(pose.position, index, *path);

    const auto [dist, index, pose] = release_point.value();
    RCLCPP_INFO_STREAM(logger, " - release line: " << index << " " << dist);
    planning_utils::insertStopPoint(pose.position, index, *path);
  */
}

}  // namespace behavior_velocity_planner::v2x_gate
