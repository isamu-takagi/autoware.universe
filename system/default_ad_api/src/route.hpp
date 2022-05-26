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

#ifndef ROUTE_HPP_
#define ROUTE_HPP_

#include "default_ad_api/specs/internal/autoware/state.hpp"
#include "default_ad_api/specs/internal/route/reset.hpp"
#include "default_ad_api/specs/internal/route/set.hpp"
#include "default_ad_api/specs/route/clear.hpp"
#include "default_ad_api/specs/route/set.hpp"
#include "default_ad_api/specs/route/state.hpp"
#include "utils/types.hpp"

#include <rclcpp/rclcpp.hpp>

namespace default_ad_api
{

class RouteNode : public rclcpp::Node
{
public:
  explicit RouteNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::CallbackGroup::SharedPtr group_;

  Publisher<ad_api::route::state::T>::SharedPtr pub_route_state_;
  Subscription<internal_api::autoware::state::T>::SharedPtr sub_autoware_state_;

  Service<ad_api::route::set::T>::SharedPtr srv_route_set_;
  Client<internal_api::route::set::T>::SharedPtr cli_route_set_;

  Service<ad_api::route::clear::T>::SharedPtr srv_route_clear_;
  Client<internal_api::route::reset::T>::SharedPtr cli_route_reset_;

  using RouteState = autoware_ad_api_msgs::msg::RouteState;
  RouteState route_state_;
};

}  // namespace default_ad_api

#endif  // ROUTE_HPP_
