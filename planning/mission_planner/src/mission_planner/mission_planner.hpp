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

#ifndef MISSION_PLANNER__MISSION_PLANNER_HPP_
#define MISSION_PLANNER__MISSION_PLANNER_HPP_

#include <component_interface_specs/planning.hpp>
#include <component_interface_utils/macros.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <mission_planner/mission_planner_plugin.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace mission_planner
{
using PoseStamped = geometry_msgs::msg::PoseStamped;
using HADMapRoute = autoware_auto_planning_msgs::msg::HADMapRoute;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using SetRoutePoints = planning_interface::SetRoutePoints;
using SetRoute = planning_interface::SetRoute;

class MissionPlanner : public rclcpp::Node
{
public:
  explicit MissionPlanner(const rclcpp::NodeOptions & options);

private:
  pluginlib::ClassLoader<MissionPlannerPlugin> plugin_loader_;
  std::shared_ptr<MissionPlannerPlugin> plugin_;
  std::string base_link_frame_;
  std::string map_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  PoseStamped getEgoVehiclePose();
  PoseStamped transformPose(const PoseStamped & input, const std::string & frame);

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;
  rclcpp::Publisher<HADMapRoute>::SharedPtr pub_route_;
  void publish(const HADMapRoute & route) const;

  component_interface_utils::Service<SetRoute>::SharedPtr srv_route_;
  component_interface_utils::Service<SetRoutePoints>::SharedPtr srv_route_points_;
  void onSetRoute(API_SERVICE_ARG(SetRoute, req, res));
  void onSetRoutePoints(API_SERVICE_ARG(SetRoutePoints, req, res));
};

}  // namespace mission_planner

#endif  // MISSION_PLANNER__MISSION_PLANNER_HPP_
