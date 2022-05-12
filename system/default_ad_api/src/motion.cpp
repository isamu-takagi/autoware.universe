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

#include "motion.hpp"

#include <component_interface_utils/response.hpp>

namespace default_ad_api
{

MotionNode::MotionNode(const rclcpp::NodeOptions & options) : Node("motion", options)
{
  using AutowareEngage = autoware_auto_vehicle_msgs::msg::Engage;
  using DrivingState = autoware_ad_api_msgs::msg::DrivingState;

  const auto on_autoware_engage = [this](MESSAGE_ARG(AutowareEngage))
  {
    // Temp
    RCLCPP_INFO_STREAM(get_logger(), "Autoware Engage: " << message->engage);
  };

  const auto on_driving_state = [this](MESSAGE_ARG(DrivingState))
  {
    // Temp
    RCLCPP_INFO_STREAM(get_logger(), "Driving State" << message->state);
  };

  const auto node = component_interface_utils::NodeAdaptor(this);
  node.init_pub(pub_motion_state_);
  node.init_cli(cli_autoware_engage_);
  node.init_sub(sub_autoware_engage_, on_autoware_engage);
  node.init_sub(sub_driving_state_, on_driving_state);
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::MotionNode)
