// Copyright 2019 Autoware Foundation
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

#ifndef TEMP__RVIZ_ADAPTOR_HPP_
#define TEMP__RVIZ_ADAPTOR_HPP_

#include <component_interface_specs/planning.hpp>
#include <component_interface_utils/macros.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>

class RvizAdaptor : public rclcpp::Node
{
public:
  explicit RvizAdaptor(const rclcpp::NodeOptions & options);

private:
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using SetRoutePoints = planning_interface::SetRoutePoints;
  SetRoutePoints::Service::Request::SharedPtr route_points_;
  component_interface_utils::Client<SetRoutePoints>::SharedPtr cli_route_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_waypoints_;
  void OnGoal(const PoseStamped::ConstSharedPtr goal_msg_ptr);
  void OnWaypoint(const PoseStamped::ConstSharedPtr checkpoint_msg_ptr);
};

#endif  // TEMP__RVIZ_ADAPTOR_HPP_
