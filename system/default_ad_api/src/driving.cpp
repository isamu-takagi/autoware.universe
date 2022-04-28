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

#include "driving.hpp"

#include <component_interface_utils/response.hpp>

namespace default_ad_api
{

DrivingNode::DrivingNode(const rclcpp::NodeOptions & options) : Node("driving", options)
{
  using DrivingEngage = autoware_ad_api_msgs::srv::DrivingEngage;
  using AutowareEngage = autoware_auto_vehicle_msgs::msg::Engage;
  using AutowareState = autoware_auto_system_msgs::msg::AutowareState;

  const auto on_driving_engage = [this](SERVICE_ARG(DrivingEngage))
  {
    RCLCPP_INFO_STREAM(get_logger(), "API Engage: " << (request->engage ? "true" : "false"));
    response->status.summary = component_interface_utils::response::success();
  };

  const auto on_autoware_engage = [this](MESSAGE_ARG(AutowareEngage))
  {
    // Temp
    RCLCPP_INFO_STREAM(get_logger(), "Autoware Engage: " << message->engage);
  };

  const auto on_autoware_state = [this](MESSAGE_ARG(AutowareState))
  {
    // Temp
    RCLCPP_INFO_STREAM(get_logger(), "Autoware State" << message->state);
  };

  const auto node = component_interface_utils::NodeAdaptor(this);
  node.init_srv(srv_api_engage_, on_driving_engage);
  node.init_pub(pub_api_state_);
  node.init_cli(cli_autoware_engage_);
  node.init_sub(sub_autoware_engage_, on_autoware_engage);
  node.init_sub(sub_autoware_state_, on_autoware_state);
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::DrivingNode)
