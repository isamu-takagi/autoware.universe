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

#include <behavior_velocity_planner/planner_data/common.hpp>
#include <utilization/util.hpp>

#include <memory>
#include <string>
#include <vector>

namespace behavior_velocity_planner::v2x_gate
{

void SceneManager::init(rclcpp::Node * node)
{
  node_ = node;
}

void SceneManager::update(const PlannerData2::ConstSharedPtr & data, const PathWithLaneId & path)
{
  const auto logger = node_->get_logger();
  data_ = data;

  static int count = 0;
  if (count++ % 10 != 0) return;

  (void)data;
  (void)path;
  RCLCPP_INFO_STREAM(logger, "v2x gate update");

  const auto map = data->common->route_handler->getLaneletMapPtr();
  const auto mapping = create_lanelet_to_v2x_gate(map);

  const auto current_pose = data->common->current_odometry->pose;
  const auto current_lane = planning_utils::getNearestLaneId(path, map, current_pose);

  std::vector<int64_t> unique_lane_ids;
  if (current_lane) {
    unique_lane_ids = planning_utils::getSubsequentLaneIdsSetOnPath(path, *current_lane);
  } else {
    unique_lane_ids = planning_utils::getSortedLaneIdsFromPath(path);
  }

  V2xGateDataSet gates;
  for (const auto & id : unique_lane_ids) {
    if (mapping.count(id)) {
      for (const auto & gate : mapping.at(id)) {
        gates.insert(gate);
      }
    }
  }

  // Create module.
  for (const auto & gate : gates) {
    const auto id = gate->gate->id();
    if (!scenes_.count(id)) {
      RCLCPP_INFO_STREAM(logger, "create scene: " << id);
      server_.create(gate->gate->getCategory(), gate->gate->id());
      scenes_.emplace(id, std::make_shared<SceneModule>(gate));
    }
  }
}

void SceneManager::plan(PathWithLaneId * path)
{
  FrameData frame;
  frame.data = data_;

  for (const auto & [lane, scene] : scenes_) {
    scene->plan(path, frame);
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
