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

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace mission_planner
{

MissionPlanner::MissionPlanner(const rclcpp::NodeOptions & options)
: Node("mission_planner", options),
  arrival_checker_(this),
  plugin_loader_("mission_planner", "mission_planner::MissionPlannerPlugin"),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  map_frame_ = declare_parameter<std::string>("map_frame");
  base_link_frame_ = declare_parameter<std::string>("base_link_frame");

  planner_ = plugin_loader_.createSharedInstance("mission_planner::lanelet2::DefaultPlanner");
  planner_->Initialize(this);

  const auto durable_qos = rclcpp::QoS(1).transient_local();
  pub_had_route_ = create_publisher<HADMapRoute>("output/route", durable_qos);
  pub_marker_ = create_publisher<MarkerArray>("debug/route_marker", durable_qos);

  const auto node = component_interface_utils::NodeAdaptor(this);
  node.init_pub(pub_state_);
  node.init_pub(pub_api_route_);
  node.init_srv(srv_clear_route_, BIND_SERVICE(this, OnClearRoute));
  node.init_srv(srv_set_route_, BIND_SERVICE(this, OnSetRoute));
  node.init_srv(srv_set_route_points_, BIND_SERVICE(this, OnSetRoutePoints));

  ChangeState(RouteState::Message::UNSET);
}

PoseStamped MissionPlanner::GetEgoVehiclePose()
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
  return TransformPose(base_link_origin);
}

PoseStamped MissionPlanner::TransformPose(const PoseStamped & input)
{
  PoseStamped output;
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(map_frame_, input.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(input, output, transform);
    return output;
  } catch (tf2::TransformException & error) {
    // TODO(Takagi, Isamu): error code
    RCLCPP_WARN_STREAM(get_logger(), error.what());
    throw component_interface_utils::ServiceException(123, error.what());
  }
}

void MissionPlanner::Publish(const HADMapRoute & route) const
{
  if (!route.segments.empty()) {
    RCLCPP_INFO(get_logger(), "Route successfully planned. Publishing...");
    // TODO(Takagi, Isamu): publish api route
    pub_had_route_->publish(route);
    pub_marker_->publish(planner_->Visualize(route));
  } else {
    RCLCPP_ERROR(get_logger(), "Calculated route is empty!");
  }
}

void MissionPlanner::ChangeState(RouteState::Message::_state_type state)
{
  state_.stamp = now();
  state_.state = state;
  pub_state_->publish(state_);
}

void MissionPlanner::OnClearRoute(API_SERVICE_ARG(ClearRoute, , res))
{
  // NOTE: The route services should be mutually exclusive by callback group.
  RCLCPP_INFO_STREAM(get_logger(), "ClearRoute");

  res->status.success = true;
  ChangeState(RouteState::Message::UNSET);
}

void MissionPlanner::OnSetRoute(API_SERVICE_ARG(SetRoute, req, res))
{
  // NOTE: The route services should be mutually exclusive by callback group.
  RCLCPP_INFO_STREAM(get_logger(), "onSetRoute");
  if (state_.state != RouteState::Message::UNSET) {
    throw component_interface_utils::ServiceException(123, "The route is already set.");
  }

  (void)req;

  res->status.success = true;
  ChangeState(RouteState::Message::SET);
}

void MissionPlanner::OnSetRoutePoints(API_SERVICE_ARG(SetRoutePoints, req, res))
{
  // NOTE: The route services should be mutually exclusive by callback group.
  RCLCPP_INFO_STREAM(get_logger(), "onSetRoutePoints");
  if (state_.state != RouteState::Message::UNSET) {
    throw component_interface_utils::ServiceException(123, "The route is already set.");
  }

  if (!planner_->Ready()) {
    // TODO(Takagi, Isamu): error code
    RCLCPP_WARN_STREAM(get_logger(), "The mission planner is not ready.");
    throw component_interface_utils::ServiceException(123, "The mission planner is not ready.");
  }

  MissionPlannerPlugin::RoutePoints points;
  PoseStamped pose;
  pose.header = req->header;

  if (req->start.empty()) {
    points.push_back(GetEgoVehiclePose());
  } else {
    pose.pose = req->start.front();
    points.push_back(TransformPose(pose));
  }
  for (const auto & waypoint : req->waypoints) {
    pose.pose = waypoint;
    points.push_back(TransformPose(pose));
  }
  pose.pose = req->goal;
  points.push_back(TransformPose(pose));

  HADMapRoute route = planner_->Plan(points);
  route.header.stamp = now();
  route.header.frame_id = map_frame_;
  Publish(route);

  res->status.success = true;
  ChangeState(RouteState::Message::SET);
}

}  // namespace mission_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mission_planner::MissionPlanner)
