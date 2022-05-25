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

#include "mission_planner/mission_planner_base.hpp"

#include <component_interface_utils/response.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>


#include <lanelet2_routing/Route.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <visualization_msgs/msg/marker_array.h>

#include <string>

namespace mission_planner
{
MissionPlanner::MissionPlanner(
  const std::string & node_name, const rclcpp::NodeOptions & node_options)
: Node(node_name, node_options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
{
  map_frame_ = declare_parameter("map_frame", "map");
  base_link_frame_ = declare_parameter("base_link_frame", "base_link");

  using std::placeholders::_1;
  using std::placeholders::_2;

  goal_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "input/goal_pose", 10, std::bind(&MissionPlanner::goalPoseCallback, this, _1));
  checkpoint_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "input/checkpoint", 10, std::bind(&MissionPlanner::checkpointCallback, this, _1));
  route_set_service_ = create_service<autoware_ad_api_msgs::srv::RouteSet>(
    "~/api/route/set", std::bind(&MissionPlanner::routeSetCallback, this, _1, _2));

  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  route_publisher_ =
    create_publisher<autoware_auto_planning_msgs::msg::HADMapRoute>("output/route", durable_qos);
  marker_publisher_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("debug/route_marker", durable_qos);
}

bool MissionPlanner::getEgoVehiclePose(geometry_msgs::msg::PoseStamped * ego_vehicle_pose)
{
  geometry_msgs::msg::PoseStamped base_link_origin;
  base_link_origin.header.frame_id = base_link_frame_;
  base_link_origin.pose.position.x = 0;
  base_link_origin.pose.position.y = 0;
  base_link_origin.pose.position.z = 0;
  base_link_origin.pose.orientation.x = 0;
  base_link_origin.pose.orientation.y = 0;
  base_link_origin.pose.orientation.z = 0;
  base_link_origin.pose.orientation.w = 1;

  //  transform base_link frame origin to map_frame to get vehicle positions
  return transformPose(base_link_origin, ego_vehicle_pose, map_frame_);
}

bool MissionPlanner::transformPose(
  const geometry_msgs::msg::PoseStamped & input_pose, geometry_msgs::msg::PoseStamped * output_pose,
  const std::string target_frame)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform =
      tf_buffer_.lookupTransform(target_frame, input_pose.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(input_pose, *output_pose, transform);
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    return false;
  }
}

void MissionPlanner::goalPoseCallback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr goal_msg_ptr)
{
  // set start pose
  if (!getEgoVehiclePose(&start_pose_)) {
    RCLCPP_ERROR(
      get_logger(), "Failed to get ego vehicle pose in map frame. Aborting mission planning");
    return;
  }
  // set goal pose
  if (!transformPose(*goal_msg_ptr, &goal_pose_, map_frame_)) {
    RCLCPP_ERROR(get_logger(), "Failed to get goal pose in map frame. Aborting mission planning");
    return;
  }

  RCLCPP_INFO(get_logger(), "New goal pose is set. Reset checkpoints.");
  checkpoints_.clear();
  checkpoints_.push_back(start_pose_);
  checkpoints_.push_back(goal_pose_);

  if (!isRoutingGraphReady()) {
    RCLCPP_ERROR(get_logger(), "RoutingGraph is not ready. Aborting mission planning");
    return;
  }

  autoware_auto_planning_msgs::msg::HADMapRoute route = planRoute();
  publishRoute(route);
}

void MissionPlanner::checkpointCallback(
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr checkpoint_msg_ptr)
{
  if (checkpoints_.size() < 2) {
    RCLCPP_ERROR(
      get_logger(),
      "You must set start and goal before setting checkpoints. Aborting mission planning");
    return;
  }

  geometry_msgs::msg::PoseStamped transformed_checkpoint;
  if (!transformPose(*checkpoint_msg_ptr, &transformed_checkpoint, map_frame_)) {
    RCLCPP_ERROR(
      get_logger(), "Failed to get checkpoint pose in map frame. Aborting mission planning");
    return;
  }

  // insert checkpoint before goal
  checkpoints_.insert(checkpoints_.end() - 1, transformed_checkpoint);

  autoware_auto_planning_msgs::msg::HADMapRoute route = planRoute();
  publishRoute(route);
}

void MissionPlanner::routeSetCallback(
  const autoware_ad_api_msgs::srv::RouteSet::Request::SharedPtr request,
  autoware_ad_api_msgs::srv::RouteSet::Response::SharedPtr response)
{
  namespace api = component_interface_utils;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  std::vector<geometry_msgs::msg::PoseStamped> transformed_waypoints;

  // set start pose
  if (request->route.start.empty())
  {
    geometry_msgs::msg::PoseStamped base_link;
    base_link.header.frame_id = base_link_frame_;
    base_link.pose.orientation.w = 1;
    waypoints.push_back(base_link);
  }
  else
  {
    geometry_msgs::msg::PoseStamped start;
    start.header = request->route.header;
    start.pose = request->route.start[0];
    waypoints.push_back(start);
  }

  // set waypoint poses
  for (const auto & pose : request->route.waypoints) {
    geometry_msgs::msg::PoseStamped waypoint;
    waypoint.header = request->route.header;
    waypoint.pose = pose;
    waypoints.push_back(waypoint);
  }

  // set goal pose
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header = request->route.header;
    goal.pose = request->route.goal;
    waypoints.push_back(goal);
  }

  // transform waypoints
  for (const auto & waypoint : waypoints) {
    geometry_msgs::msg::PoseStamped transformed;
    if (!transformPose(waypoint, &transformed, map_frame_)) {
      response->status = api::response::error(0, "Failed to transform waypoints.");
      return;
    }
    transformed_waypoints.push_back(transformed);
  }

  RCLCPP_INFO(get_logger(), "New route is set. Reset checkpoints.");
  start_pose_ = transformed_waypoints.front();
  goal_pose_ = transformed_waypoints.back();
  checkpoints_ = transformed_waypoints;

  if (!isRoutingGraphReady()) {
    response->status = api::response::error(0, "RoutingGraph is not ready.");
    return;
  }

  autoware_auto_planning_msgs::msg::HADMapRoute route = planRoute();
  publishRoute(route);
  response->status = api::response::success();
}

void MissionPlanner::publishRoute(const autoware_auto_planning_msgs::msg::HADMapRoute & route) const
{
  if (!route.segments.empty()) {
    RCLCPP_INFO(get_logger(), "Route successfully planned. Publishing...");
    route_publisher_->publish(route);
    visualizeRoute(route);
  } else {
    RCLCPP_ERROR(get_logger(), "Calculated route is empty!");
  }
}

}  // namespace mission_planner
