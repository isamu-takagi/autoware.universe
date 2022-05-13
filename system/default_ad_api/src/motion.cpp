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

#include <memory>

namespace default_ad_api
{

MotionNode::MotionNode(const rclcpp::NodeOptions & options) : Node("motion", options)
{
  using AutowareEngage = autoware_auto_vehicle_msgs::msg::Engage;
  const auto on_autoware_engage = [this](MESSAGE_ARG(AutowareEngage))
  {
    (void)message;
    // Temp
    // RCLCPP_INFO_STREAM(get_logger(), "Autoware Engage: " << message->engage);
  };

  using DrivingState = autoware_ad_api_msgs::msg::DrivingState;
  const auto on_driving_state = [this](MESSAGE_ARG(DrivingState))
  {
    if (driving_state_.state != message->state) {
      RCLCPP_INFO_STREAM(get_logger(), "Driving State" << message->state);

      using EngageRequest = tier4_external_api_msgs::srv::Engage::Request;
      const auto service_callback =
        [this](rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedFuture future)
      {
        RCLCPP_INFO_STREAM(
          get_logger(), future.get()->status.code << " " << future.get()->status.message);
      };

      if (message->state == DrivingState::DRIVING) {
        const auto request = std::make_shared<EngageRequest>();
        request->engage = true;
        cli_autoware_engage_->async_send_request(request, service_callback);
      } else {
        const auto request = std::make_shared<EngageRequest>();
        request->engage = false;
        cli_autoware_engage_->async_send_request(request, service_callback);
      }
    }
    driving_state_ = *message;
  };

  const auto node = component_interface_utils::NodeAdaptor(this);
  node.init_pub(pub_motion_state_);
  node.init_cli(cli_autoware_engage_);
  node.init_sub(sub_autoware_engage_, on_autoware_engage);
  node.init_sub(sub_driving_state_, on_driving_state);

  // initialize state machine
  {
    using MotionState = autoware_ad_api_msgs::msg::MotionState;
    temp_state_ = MotionState::STOPPED;

    driving_state_.state = DrivingState::UNKNOWN;
  }
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::MotionNode)
