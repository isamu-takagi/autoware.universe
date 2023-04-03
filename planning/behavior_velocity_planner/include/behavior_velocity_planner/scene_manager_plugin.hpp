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

#ifndef BEHAVIOR_VELOCITY_PLANNER__SCENE_MANAGER_PLUGIN_HPP_
#define BEHAVIOR_VELOCITY_PLANNER__SCENE_MANAGER_PLUGIN_HPP_

#include <behavior_velocity_planner/planner_data.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/primitives/Primitive.h>

#include <memory>
#include <unordered_map>

namespace behavior_velocity_planner
{

struct PlannerData2
{
  using ConstSharedPtr = std::shared_ptr<const PlannerData2>;
};

class SceneManagerPlugin
{
public:
  using PathWithLaneId = autoware_auto_planning_msgs::msg::PathWithLaneId;

  virtual ~SceneManagerPlugin() = default;
  virtual void init(rclcpp::Node * node) = 0;
  // virtual void update(const std::shared_ptr<const PlannerData> & data, const PathWithLaneId &
  // path) = 0;
  virtual void plan(PathWithLaneId * path) = 0;

  // TODO(Takagi, Isamu): use std::optional
  virtual boost::optional<int> getFirstStopPathPointIndex() = 0;

  // TODO(Takagi, Isamu): use std::string
  virtual const char * getModuleName() = 0;

  // TODO(Takagi, Isamu): use update function
  virtual void updateSceneModuleInstances(
    const std::shared_ptr<const PlannerData> &, const PathWithLaneId &)
  {
  }
};

}  // namespace behavior_velocity_planner

#endif  // BEHAVIOR_VELOCITY_PLANNER__SCENE_MANAGER_PLUGIN_HPP_
