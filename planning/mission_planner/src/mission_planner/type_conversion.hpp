// Copyright 2022 Autoware Foundation
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

#ifndef MISSION_PLANNER__TYPE_CONVERSION_HPP_
#define MISSION_PLANNER__TYPE_CONVERSION_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_ad_api_msgs/msg/route.hpp>
#include <autoware_ad_api_msgs/msg/route_optional.hpp>
#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>

namespace mission_planner::conversion
{

using APIRouteBody = autoware_ad_api_msgs::msg::Route;
using APIRoute = autoware_ad_api_msgs::msg::RouteOptional;
using HADRoute = autoware_auto_planning_msgs::msg::HADMapRoute;

APIRoute CreateEmptyRoute(const rclcpp::Time & stamp);
APIRoute ConvertRoute(const HADRoute & api_route);
HADRoute ConvertRoute(const std_msgs::msg::Header & header, const APIRouteBody & body);

}  // namespace mission_planner::conversion

#endif  // MISSION_PLANNER__TYPE_CONVERSION_HPP_
