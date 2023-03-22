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

#include "manager.hpp"

#include "lanelet.hpp"
#include "utilization/util.hpp"

#include <memory>
#include <string>
#include <vector>

namespace behavior_v2x_gate
{

void SceneManager::init(rclcpp::Node * node) { node_ = node; }

void SceneManager::updateSceneModuleInstances(
  const std::shared_ptr<const PlannerData> & data, const PathWithLaneId & path)
{
  namespace planning_utils = behavior_velocity_planner::planning_utils;

  static int count = 0;
  if (count++ % 10 != 0) return;

  (void)data;
  (void)path;
  RCLCPP_INFO_STREAM(node_->get_logger(), "v2x gate update");

  const auto & gates = get_all_v2x_gates(data->route_handler_->getLaneletMapPtr());
  for (const auto & gate : gates) {
    RCLCPP_INFO_STREAM(node_->get_logger(), " - gate: " << gate->id());
  }

  const auto & map = data->route_handler_->getLaneletMapPtr();
  const auto & current_pose = data->current_odometry->pose;

  const auto current_lane = planning_utils::getNearestLaneId(path, map, current_pose);

  std::vector<int64_t> unique_lane_ids;
  if (current_lane) {
    unique_lane_ids = planning_utils::getSubsequentLaneIdsSetOnPath(path, *current_lane);
  } else {
    unique_lane_ids = planning_utils::getSortedLaneIdsFromPath(path);
  }

  std::string ids;
  for (const auto & id : unique_lane_ids) {
    ids += " " + std::to_string(id);
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), " - path:" << ids);
}

void SceneManager::plan(PathWithLaneId * path) { (void)path; }

boost::optional<int> SceneManager::getFirstStopPathPointIndex() { return boost::none; }

const char * SceneManager::getModuleName() { return ""; }

}  // namespace behavior_v2x_gate

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  behavior_v2x_gate::SceneManager, behavior_velocity_planner::SceneManagerPlugin)
