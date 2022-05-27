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
    const auto on_driving_engage = [this](SERVICE_ARG(DrivingEngage)) {
      using DrivingState = autoware_ad_api_msgs::msg::DrivingState;
      if (request->engage) {
        if (driving_state_.state == DrivingState::READY) {
          driving_state_.state = DrivingState::DRIVING;
        }
      } else {
        driving_state_.state = DrivingState::PREPARING;
      }
      response->status = component_interface_utils::response_success();
    };

    using AutowareState = autoware_auto_system_msgs::msg::AutowareState;
    const auto on_autoware_state = [this](MESSAGE_ARG(AutowareState)) {
      using DrivingState = autoware_ad_api_msgs::msg::DrivingState;
      const auto prev_state = driving_state_;

      switch (message->state) {
        case AutowareState::WAITING_FOR_ENGAGE:
          if (driving_state_.state == DrivingState::PREPARING) {
            driving_state_.state = DrivingState::READY;
          }
          break;
        case AutowareState::DRIVING:
          driving_state_.state = DrivingState::DRIVING;
          break;
        default:
          driving_state_.state = DrivingState::PREPARING;
          break;
      }

      if (driving_state_ != prev_state) {
        pub_driving_state_->publish(driving_state_);
      }
    };

    const auto node = component_interface_utils::NodeAdaptor(this);
    node.init_srv(srv_driving_engage_, on_driving_engage);
    node.init_pub(pub_driving_state_);
    node.init_sub(sub_autoware_state_, on_autoware_state);
  }

  // initialize state machine
  {
    using DrivingState = autoware_ad_api_msgs::msg::DrivingState;
    driving_state_.state = DrivingState::PREPARING;
  }
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::DrivingNode)
