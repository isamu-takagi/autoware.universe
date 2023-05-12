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

#include <string>
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

std::set<std::string> get_line_ids(const LaneToLines & lines)
{
  std::set<std::string> result;
  for (const auto & [lane, line] : lines) {
    result.insert(std::to_string(line.id()));
  }
  return result;
}

SceneModule::SceneModule(const GateArea::ConstSharedPtr & gate)
{
  gate_ = gate;
  lock_ = std::make_unique<LockTarget>(gate->getCategory(), gate->id());
}

void SceneModule::plan(PathWithLaneId * path, const FrameData & frame)
{
  using arc_lane_utils::PathIndexWithPose;

  const auto logger = rclcpp::get_logger("behavior_velocity_planner.v2x_gate");
  RCLCPP_INFO_STREAM(logger, "scene module: " << gate_->id());

  // Get gate lines on the path.
  const auto acquire_lines = filter_lines(gate_->getAcquireLines(), frame.lane_ids_on_path);
  const auto release_lines = filter_lines(gate_->getReleaseLines(), frame.lane_ids_on_path);

  // Get vehicle position
  const auto vehicle_point = find_ego_segment_index(path->points, frame.data);
  const auto acquire_point = get_first_cross_point(*path, frame, vehicle_point, acquire_lines);
  const auto release_point = get_first_cross_point(*path, frame, vehicle_point, release_lines);

  if (acquire_point) {
    const auto & point = acquire_point.value();
    if (0.0 < point.distance) {
      const auto status = lock_->status();
      if (!status.lines.count(std::to_string(point.line.id()))) {
        const auto lines = get_union(get_line_ids(acquire_lines), get_line_ids(release_lines));
        lock_->update(lines, point.distance);
        planning_utils::insertStopPoint(point.pose.position, point.index, *path);
      }
      return;
    }
  }

  if (release_point) {
    const auto & point = release_point.value();
    if (0.0 < point.distance) {
      planning_utils::insertStopPoint(point.pose.position, point.index, *path);
      return;
    }
  }
}

}  // namespace behavior_velocity_planner::v2x_gate
