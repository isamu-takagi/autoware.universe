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

#ifndef BEHAVIOR_VELOCITY_PLANNER__PLANNER_DATA__COMMON_HPP_
#define BEHAVIOR_VELOCITY_PLANNER__PLANNER_DATA__COMMON_HPP_

#include <route_handler/route_handler.hpp>
#include <vehicle_info_util/vehicle_info.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>

namespace behavior_velocity_planner
{

struct PlannerDataCommon
{
  std::shared_ptr<route_handler::RouteHandler> route_handler;
  vehicle_info_util::VehicleInfo vehicle_info;

  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_odometry;

  double stop_line_extend_length;
  double ego_nearest_dist_threshold;
  double ego_nearest_yaw_threshold;
};

}  // namespace behavior_velocity_planner

#endif  // BEHAVIOR_VELOCITY_PLANNER__PLANNER_DATA__COMMON_HPP_
