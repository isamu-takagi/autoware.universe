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

#include "route.hpp"

namespace default_ad_api
{

RouteNode::RouteNode(const rclcpp::NodeOptions & options) : Node("route", options)
{
  using AutowareState = autoware_auto_system_msgs::msg::AutowareState;
  const auto on_autoware_state = [this](MESSAGE_ARG(AutowareState)) {
    // TODO(Takagi, Isamu): This is a temporary logic, use the component interface later.
    RouteState prev_state = route_state_;

    switch (message->state) {
      case AutowareState::WAITING_FOR_ENGAGE:
      case AutowareState::DRIVING:
        if (route_state_.state == RouteState::UNSET) {
          route_state_.state = RouteState::SET;
        }
        break;

      case AutowareState::ARRIVED_GOAL:
        if (route_state_.state == RouteState::SET) {
          route_state_.state = RouteState::ARRIVED;
        }
        break;
    }
    route_state_.is_planning = (message->state == AutowareState::PLANNING);

    if (route_state_ != prev_state) {
      pub_route_state_->publish(route_state_);
    }
  };

  const auto node = component_interface_utils::NodeAdaptor(this);
  node.init_pub(pub_route_state_);
  node.init_sub(sub_autoware_state_, on_autoware_state);

  route_state_.state = RouteState::UNSET;
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::RouteNode)
