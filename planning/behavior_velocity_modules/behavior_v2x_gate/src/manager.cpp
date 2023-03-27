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
  const auto logger = node_->get_logger();

  static int count = 0;
  if (count++ % 10 != 0) return;

  (void)data;
  (void)path;
  RCLCPP_INFO_STREAM(logger, "v2x gate update");

  const auto map = data->route_handler_->getLaneletMapPtr();
  const auto mapping = create_lanelet_to_v2x_gate(map);

  const auto current_pose = data->current_odometry->pose;
  const auto current_lane = planning_utils::getNearestLaneId(path, map, current_pose);

  std::vector<int64_t> unique_lane_ids;
  if (current_lane) {
    unique_lane_ids = planning_utils::getSubsequentLaneIdsSetOnPath(path, *current_lane);
  } else {
    unique_lane_ids = planning_utils::getSortedLaneIdsFromPath(path);
  }

  V2xGateDataSet gates;
  for (const auto & id : unique_lane_ids) {
    RCLCPP_INFO_STREAM(logger, "Lane " << id);
    if (mapping.count(id)) {
      for (const auto & gate : mapping.at(id)) {
        RCLCPP_INFO_STREAM(logger, "  Gate " << gate->gate->id());
        gates.insert(gate);
      }
    }
  }

  // const auto gates = get_all_v2x_gates(data->route_handler_->getLaneletMapPtr());
  /*
  const auto gates = planning_utils::getRegElemMapOnPath<V2xGate>(path, map, current_pose);
  for (const auto & gate : gates) {
    RCLCPP_INFO_STREAM(node_->get_logger(), " - gate: " << gate.first->id() << " " <<
  gate.first->id());
  }
  */

  /*
  for (const auto & m : planning_utils::getRegElemMapOnPath<VirtualTrafficLight>(
         path, planner_data_->route_handler_->getLaneletMapPtr(),
         planner_data_->current_odometry->pose)) {
    // Use lanelet_id to unregister module when the route is changed
    const auto lane_id = m.second.id();
    const auto module_id = lane_id;
    if (!isModuleRegistered(module_id)) {
      registerModule(std::make_shared<VirtualTrafficLightModule>(
        module_id, lane_id, *m.first, m.second, planner_param_,
        logger_.get_child("virtual_traffic_light_module"), clock_));
    }
  }
  */
}

void SceneManager::plan(PathWithLaneId * path) { (void)path; }

boost::optional<int> SceneManager::getFirstStopPathPointIndex() { return boost::none; }

const char * SceneManager::getModuleName() { return ""; }

}  // namespace behavior_v2x_gate

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  behavior_v2x_gate::SceneManager, behavior_velocity_planner::SceneManagerPlugin)
