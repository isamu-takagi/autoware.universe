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

/*
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
*/

#include "rviz_adaptor.hpp"

#include <memory>
#include <string>

RvizAdaptor::RvizAdaptor(const rclcpp::NodeOptions & options)
: Node("rviz_routing_adaptor", options)
{
  RCLCPP_INFO_STREAM(get_logger(), "RvizAdaptor");

  sub_goal_ = create_subscription<PoseStamped>(
    "input/goal_pose", 5, std::bind(&RvizAdaptor::OnGoal, this, std::placeholders::_1));
  sub_waypoints_ = create_subscription<PoseStamped>(
    "input/checkpoint", 5, std::bind(&RvizAdaptor::OnWaypoint, this, std::placeholders::_1));

  const auto node = component_interface_utils::NodeAdaptor(this);
  node.init_cli(cli_route_);
  route_points_ = std::make_shared<SetRoutePoints::Service::Request>();
}

void RvizAdaptor::OnGoal(const PoseStamped::ConstSharedPtr pose)
{
  RCLCPP_INFO_STREAM(get_logger(), "OnGoal");
  route_points_->header = pose->header;
  route_points_->goal = pose->pose;
  route_points_->waypoints.clear();
  cli_route_->async_send_request(route_points_);

  // set start pose
  /*
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
  */
}

void RvizAdaptor::OnWaypoint(const PoseStamped::ConstSharedPtr pose)
{
  (void)pose;
  RCLCPP_INFO_STREAM(get_logger(), "OnWaypoint");

  /*
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
  */
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(RvizAdaptor)
