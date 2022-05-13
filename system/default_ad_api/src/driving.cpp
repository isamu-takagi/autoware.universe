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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <component_interface_utils/response.hpp>

namespace default_ad_api
{

DrivingNode::DrivingNode(const rclcpp::NodeOptions & options) : Node("driving", options)
{
  // initialize ros interface
  {
    using DrivingEngage = autoware_ad_api_msgs::srv::DrivingEngage;
    const auto on_driving_engage = [this](SERVICE_ARG(DrivingEngage))
    {
      using DrivingState = autoware_ad_api_msgs::msg::DrivingState;
      RCLCPP_INFO_STREAM(get_logger(), "API Engage: " << (request->engage ? "true" : "false"));

      if (request->engage) {
        if (temp_state_ == DrivingState::READY) {
          temp_state_ = DrivingState::DRIVING;
        }
      } else {
        temp_state_ = DrivingState::PREPARING;
      }
      response->status.summary = component_interface_utils::response::success();
    };

    using AutowareState = autoware_auto_system_msgs::msg::AutowareState;
    const auto on_autoware_state = [this](MESSAGE_ARG(AutowareState))
    {
      if (autoware_state_.state != message->state) {
        RCLCPP_INFO_STREAM(get_logger(), "Autoware State" << static_cast<int>(message->state));
      }
      autoware_state_ = *message;

      using DrivingState = autoware_ad_api_msgs::msg::DrivingState;
      switch (message->state) {
        case AutowareState::WAITING_FOR_ENGAGE:
          if (temp_state_ == DrivingState::PREPARING) {
            temp_state_ = DrivingState::READY;
          }
          break;
        case AutowareState::DRIVING:
          temp_state_ = DrivingState::DRIVING;
          break;
        default:
          temp_state_ = DrivingState::PREPARING;
          break;
      }

      // TODO(Takagi, Isamu): callback from state machine
      DrivingState msg;
      msg.state = temp_state_;
      pub_driving_state_->publish(msg);
    };

    const auto node = component_interface_utils::NodeAdaptor(this);
    node.init_srv(srv_driving_engage_, on_driving_engage);
    node.init_pub(pub_driving_state_);
    node.init_sub(sub_autoware_state_, on_autoware_state);
  }

  // initialize state machine
  {
    using DrivingState = autoware_ad_api_msgs::msg::DrivingState;
    temp_state_ = DrivingState::PREPARING;
    /*
    const auto path = ament_index_cpp::get_package_share_directory("default_ad_api");
    component_state_machine::StateMachineLoader loader;
    loader.BindState(DrivingState::PREPARING, "preparing");
    loader.BindState(DrivingState::READY, "ready");
    loader.BindState(DrivingState::DRIVING, "driving");
    loader.LoadYAML(driving_state_machine_, path + "/state/driving.yaml");
    */

    using AutowareState = autoware_auto_system_msgs::msg::AutowareState;
    autoware_state_.state = AutowareState::INITIALIZING;
  }
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::DrivingNode)
