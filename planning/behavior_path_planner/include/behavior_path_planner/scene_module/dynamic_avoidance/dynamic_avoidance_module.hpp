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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__DYNAMIC_AVOIDANCE__DYNAMIC_AVOIDANCE_MODULE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__DYNAMIC_AVOIDANCE__DYNAMIC_AVOIDANCE_MODULE_HPP_

#include "behavior_path_planner/scene_module/scene_module_interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg_array.hpp>

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
struct DynamicAvoidanceParameters
{
  // obstacle types to avoid
  bool avoid_car{true};
  bool avoid_truck{true};
  bool avoid_bus{true};
  bool avoid_trailer{true};
  bool avoid_unknown{false};
  bool avoid_bicycle{false};
  bool avoid_motorcycle{false};
  bool avoid_pedestrian{false};
  double min_obstacle_vel{0.0};

  // drivable area generation
  double lat_offset_from_obstacle{0.0};
  double time_to_avoid_same_directional_object{0.0};
  double time_to_avoid_opposite_directional_object{0.0};
  double max_lat_offset_to_avoid{0.0};
};

class DynamicAvoidanceModule : public SceneModuleInterface
{
public:
  struct DynamicAvoidanceObject
  {
    explicit DynamicAvoidanceObject(
      const PredictedObject & predicted_object, const double arg_path_projected_vel)
    : pose(predicted_object.kinematics.initial_pose_with_covariance.pose),
      path_projected_vel(arg_path_projected_vel),
      shape(predicted_object.shape)
    {
    }

    geometry_msgs::msg::Pose pose;
    double path_projected_vel;
    autoware_auto_perception_msgs::msg::Shape shape;
  };

#ifdef USE_OLD_ARCHITECTURE
  DynamicAvoidanceModule(
    const std::string & name, rclcpp::Node & node,
    std::shared_ptr<DynamicAvoidanceParameters> parameters);
#else
  DynamicAvoidanceModule(
    const std::string & name, rclcpp::Node & node,
    std::shared_ptr<DynamicAvoidanceParameters> parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface> > & rtc_interface_ptr_map);

  void updateModuleParams(const std::shared_ptr<DynamicAvoidanceParameters> & parameters)
  {
    parameters_ = parameters;
  }
#endif

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  ModuleStatus updateState() override;
  ModuleStatus getNodeStatusWhileWaitingApproval() const override { return ModuleStatus::SUCCESS; }
  BehaviorModuleOutput plan() override;
  CandidateOutput planCandidate() const override;
  BehaviorModuleOutput planWaitingApproval() override;
  void updateData() override;
  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

private:
  std::vector<DynamicAvoidanceObject> calcTargetObjects() const;
  lanelet::ConstLanelets getAdjacentLanes(
    const double forward_distance, const double backward_distance) const;
  std::optional<tier4_autoware_utils::Polygon2d> calcDynamicObstaclePolygon(
    const PathWithLaneId & path, const DynamicAvoidanceObject & object) const;

  std::vector<DynamicAvoidanceModule::DynamicAvoidanceObject> target_objects_;
  std::shared_ptr<DynamicAvoidanceParameters> parameters_;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__DYNAMIC_AVOIDANCE__DYNAMIC_AVOIDANCE_MODULE_HPP_
