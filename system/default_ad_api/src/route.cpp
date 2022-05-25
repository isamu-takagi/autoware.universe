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

#include <component_interface_utils/response.hpp>

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

  using RouteSet = autoware_ad_api_msgs::srv::RouteSet;
  const auto on_route_set = [this](SERVICE_ARG(RouteSet)) {
    namespace api = component_interface_utils;
    if (route_state_.state != RouteState::UNSET || route_state_.is_planning) {
      response->status = api::response::error(0, "invalid state");
      return;
    }
    const auto res = cli_route_set_->call(request);
    response->status = res->status;
  };

  const auto group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  group_ = group;
  const auto node = component_interface_utils::NodeAdaptor(this);
  node.init_pub(pub_route_state_);
  node.init_sub(sub_autoware_state_, on_autoware_state);
  node.init_cli(cli_route_set_);
  node.init_srv(srv_route_set_, on_route_set, group);

  route_state_.state = RouteState::UNSET;
  pub_route_state_->publish(route_state_);
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::RouteNode)
