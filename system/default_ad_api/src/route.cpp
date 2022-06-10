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

#include <memory>

namespace default_ad_api
{

RouteNode::RouteNode(const rclcpp::NodeOptions & options) : Node("route", options)
{
  // TODO(Takagi, Isamu): This is a temporary logic, use the component interface later.

  using component_interface_utils::response_error;
  using component_interface_utils::response_success;
  using component_interface_utils::response_warning;

  using AutowareState = autoware_auto_system_msgs::msg::AutowareState;
  const auto on_autoware_state = [this](MESSAGE_ARG(AutowareState)) {
    if (message->state == AutowareState::ARRIVED_GOAL) {
      UpdateRouteState(RouteState::ARRIVED);
    }
  };

  using RouteSet = autoware_ad_api_msgs::srv::RouteSet;
  const auto on_route_set = [this](SERVICE_ARG(RouteSet)) {
    if (route_state_.state != RouteState::UNSET) {
      response->status = response_error(0, "invalid state");
      return;
    }

    const auto res = cli_route_set_->call(request);
    if (!res->status.success) {
      response->status = res->status;
      return;
    }

    UpdateRouteState(RouteState::SET);
    response->status = res->status;
  };

  using RouteClear = autoware_ad_api_msgs::srv::RouteClear;
  const auto on_route_clear = [this](SERVICE_ARG_NO_REQ(RouteClear)) {
    if (route_state_.state == RouteState::UNSET) {
      response->status = response_warning(0, "invalid state");
      return;
    }

    UpdateRouteState(RouteState::UNSET);
    response->status = response_success();
  };

  const auto node = component_interface_utils::NodeAdaptor(this);
  group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  node.init_pub(pub_route_state_);
  node.init_sub(sub_autoware_state_, on_autoware_state);
  node.init_cli(cli_route_set_);
  node.init_srv(srv_route_set_, on_route_set, group_);
  node.init_srv(srv_route_clear_, on_route_clear, group_);

  // Initial setting. Do not use UpdateRouteState.
  route_state_.state = RouteState::UNSET;
  pub_route_state_->publish(route_state_);
}

void RouteNode::UpdateRouteState(RouteState::_state_type state)
{
  if (route_state_.state == state) {
    return;
  }
  route_state_.state = state;
  pub_route_state_->publish(route_state_);
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::RouteNode)
