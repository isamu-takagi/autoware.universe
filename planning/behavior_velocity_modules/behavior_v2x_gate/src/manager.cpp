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

#include <utilization/util.hpp>

#include <memory>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

namespace
{

/*
std::set<lanelet::Id> get_lane_ids_on_path(const PathWithLaneId & path)
{
  std::set<lanelet::Id> lane_ids;
  for (const auto & points : path.points) {
    for (const auto lane_id : points.lane_ids) {
      lane_ids.insert(lane_id);
    }
  }
  return lane_ids;
}
*/

}  // namespace

namespace behavior_velocity_planner::v2x_gate
{

void SceneManager::init(rclcpp::Node * node)
{
  node_ = node;
  server_ = std::make_unique<LockServer>(node);
}

void SceneManager::update(const PlannerData2::ConstSharedPtr & data, const PathWithLaneId &)
{
  data_ = data;
}

void SceneManager::plan(PathWithLaneId * path)
{
  const auto logger = node_->get_logger();
  RCLCPP_INFO_STREAM(logger, "v2x gate plan");

  const auto map = data_->route_handler->getLaneletMapPtr();
  const auto mapping = get_lane_to_gate_areas(map);

  const auto current_pose = data_->current_odometry->pose;
  const auto current_lane = planning_utils::getNearestLaneId(*path, map, current_pose);

  std::vector<int64_t> lane_ids_on_path;
  if (current_lane) {
    lane_ids_on_path = planning_utils::getSubsequentLaneIdsSetOnPath(*path, *current_lane);
  } else {
    lane_ids_on_path = planning_utils::getSortedLaneIdsFromPath(*path);
  }

  std::unordered_set<GateArea::ConstSharedPtr> gates_on_route;
  for (const auto & id : lane_ids_on_path) {
    if (mapping.count(id)) {
      for (const auto & gate : mapping.at(id)) {
        gates_on_route.insert(gate);
      }
    }
  }

  // Create module.
  for (const auto & gate : gates_on_route) {
    const auto id = gate->id();
    if (!scenes_.count(id)) {
      RCLCPP_INFO_STREAM(logger, "create scene: " << id);
      scenes_.emplace(id, std::make_shared<SceneModule>(gate));
    }
  }

  FrameData frame;
  frame.data = data_;
  frame.lane_ids_on_path = std::set<lanelet::Id>(lane_ids_on_path.begin(), lane_ids_on_path.end());

  for (const auto & [lane, scene] : scenes_) {
    scene->plan(path, frame);
  }

  // Synchronize status
  for (const auto & [lane, scene] : scenes_) {
    scene->lock().update(*server_);
  }

  data_ = nullptr;
}

boost::optional<int> SceneManager::getFirstStopPathPointIndex()
{
  return boost::none;
}

const char * SceneManager::getModuleName()
{
  return "v2x_gate";
}

}  // namespace behavior_velocity_planner::v2x_gate

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  behavior_velocity_planner::v2x_gate::SceneManager, behavior_velocity_planner::SceneManagerPlugin)
