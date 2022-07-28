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

#include "mission_planner.hpp"

#include <string>

namespace mission_planner
{
MissionPlanner::MissionPlanner(const std::string & name, const rclcpp::NodeOptions & options)
: Node(name, options),
  plugin_loader_("mission_planner", "mission_planner::MissionPlannerPlugin"),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  map_frame_ = declare_parameter("map_frame", "map");
  base_link_frame_ = declare_parameter("base_link_frame", "base_link");

  const auto durable_qos = rclcpp::QoS(1).transient_local();
  pub_route_ = create_publisher<HADMapRoute>("output/route", durable_qos);
  pub_marker_ = create_publisher<MarkerArray>("debug/route_marker", durable_qos);

  const auto node = component_interface_utils::NodeAdaptor(this);
  node.init_srv(srv_route_points_, BIND_SERVICE(this, onSetRoutePoints));
  node.init_srv(srv_route_, BIND_SERVICE(this, onSetRoute));
}

PoseStamped MissionPlanner::getEgoVehiclePose()
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
  return transformPose(base_link_origin, map_frame_);
}

PoseStamped MissionPlanner::transformPose(const PoseStamped & input, const std::string & frame)
{
  PoseStamped output;
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(frame, input.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(input, output, transform);
    return output;
  } catch (tf2::TransformException & error) {
    // TODO(Takagi, Isamu): error code
    RCLCPP_WARN_STREAM(get_logger(), error.what());
    throw component_interface_utils::ServiceException(123, error.what());
  }
}

void MissionPlanner::publish(const HADMapRoute & route) const
{
  if (!route.segments.empty()) {
    RCLCPP_INFO(get_logger(), "Route successfully planned. Publishing...");
    pub_route_->publish(route);
    pub_marker_->publish(plugin_->visualize(route));
  } else {
    RCLCPP_ERROR(get_logger(), "Calculated route is empty!");
  }
}

// API_SERVICE_ARG
void MissionPlanner::onSetRoute(API_SERVICE_ARG(SetRoute, req, res))
{
  (void)req;
  (void)res;
}

void MissionPlanner::onSetRoutePoints(API_SERVICE_ARG(SetRoutePoints, req, res))
{
  (void)req;
  (void)res;

  /*
  using component_interface_utils::response_error;
  using component_interface_utils::response_success;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  std::vector<geometry_msgs::msg::PoseStamped> transformed_waypoints;

  // set start pose
  if (request->start.empty()) {
    geometry_msgs::msg::PoseStamped base_link;
    base_link.header.frame_id = base_link_frame_;
    base_link.pose.orientation.w = 1;
    waypoints.push_back(base_link);
  } else {
    geometry_msgs::msg::PoseStamped start;
    start.header = request->header;
    start.pose = request->start[0];
    waypoints.push_back(start);
  }

  // set waypoint poses
  for (const auto & pose : request->waypoints) {
    geometry_msgs::msg::PoseStamped waypoint;
    waypoint.header = request->header;
    waypoint.pose = pose;
    waypoints.push_back(waypoint);
  }

  // set goal pose
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header = request->header;
    goal.pose = request->goal;
    waypoints.push_back(goal);
  }

  // transform waypoints
  for (const auto & waypoint : waypoints) {
    geometry_msgs::msg::PoseStamped transformed;
    if (!transformPose(waypoint, &transformed, map_frame_)) {
      response->status = response_error(0, "Failed to transform waypoints.");
      return;
    }
    transformed_waypoints.push_back(transformed);
  }

  RCLCPP_INFO(get_logger(), "New route is set. Reset checkpoints.");
  start_pose_ = transformed_waypoints.front();
  goal_pose_ = transformed_waypoints.back();
  checkpoints_ = transformed_waypoints;

  if (!isRoutingGraphReady()) {
    response->status = response_error(0, "RoutingGraph is not ready.");
    return;
  }

  autoware_auto_planning_msgs::msg::HADMapRoute route = planRoute();
  publishRoute(route);
  response->status = response_success();
  */
}

}  // namespace mission_planner
