// Copyright 2023 TIER IV, Inc.
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

#ifndef SCENE_MODULE__OUT_OF_LANE__SCENE_OUT_OF_LANE_HPP_
#define SCENE_MODULE__OUT_OF_LANE__SCENE_OUT_OF_LANE_HPP_

#include "scene_module/out_of_lane/types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <scene_module/scene_module_interface.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <string>
#include <vector>

namespace behavior_velocity_planner::out_of_lane
{
class OutOfLaneModule : public SceneModuleInterface
{
public:
  OutOfLaneModule(
    const int64_t module_id, PlannerParam planner_param, const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr clock);

  /// @brief insert stop or slow down points to prevent dangerously entering another lane
  /// @param [inout] path the path to update
  /// @param [inout] stop_reason reason for stopping
  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  visualization_msgs::msg::MarkerArray createVirtualWallMarkerArray() override;

private:
  // Parameter
  PlannerParam params_;

protected:
  int64_t module_id_{};

  // Debug
  mutable DebugData debug_data_;

  std::shared_ptr<motion_utils::VirtualWallMarkerCreator> virtual_wall_marker_creator_ =
    std::make_shared<motion_utils::VirtualWallMarkerCreator>();
};

/// @brief calculate points along the path where we want ego to slowdown/stop
/// @param ego_data ego data (path, velocity, etc)
/// @param decisions decisions (before which point to stop, what lane to avoid entering, etc)
/// @param params parameters
/// @return precise points to insert in the path
std::vector<SlowdownToInsert> calculate_slowdown_points(
  const EgoData & ego_data, const std::vector<Slowdown> & decisions, PlannerParam params);

}  // namespace behavior_velocity_planner::out_of_lane

#endif  // SCENE_MODULE__OUT_OF_LANE__SCENE_OUT_OF_LANE_HPP_
