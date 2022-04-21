// Copyright 2022 TIER IV, Inc.
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

#include "default_ad_api/nodes/driving.hpp"

#include <component_interface_utils/response.hpp>

namespace default_ad_api
{

DrivingNode::DrivingNode(const rclcpp::NodeOptions & options) : Node("driving", options)
{
  srv_api_engage_ = component_interface_utils::create_service<ad_api::driving::engage::T>(
    this, &DrivingNode::onDrivingEngage);

  pub_api_state_ = component_interface_utils::create_publisher<ad_api::driving::state::T>(this);

  cli_autoware_engage_ =
    component_interface_utils::create_client<internal_api::autoware::set_engage::T>(this);

  sub_autoware_engage_ =
    component_interface_utils::create_subscription<internal_api::autoware::get_engage::T>(
      this, &DrivingNode::onAutowareEngage);

  sub_autoware_state_ =
    component_interface_utils::create_subscription<internal_api::autoware::get_state::T>(
      this, &DrivingNode::onAutowareState);
}

void DrivingNode::onDrivingEngage(
  const DrivingEngage::Request::SharedPtr request,
  const DrivingEngage::Response::SharedPtr response)
{
  RCLCPP_INFO(get_logger(), "engage: %s", request->engage ? "true" : "false");
  response->status.summary = component_interface_utils::response::success();
}

void DrivingNode::onAutowareEngage(
  const autoware_auto_vehicle_msgs::msg::Engage::ConstSharedPtr message)
{
  (void)message;
  RCLCPP_INFO(get_logger(), "onAutowareEngage");
}

void DrivingNode::onAutowareState(
  const autoware_auto_system_msgs::msg::AutowareState::ConstSharedPtr message)
{
  (void)message;
  RCLCPP_INFO(get_logger(), "onAutowareState");
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::DrivingNode)
