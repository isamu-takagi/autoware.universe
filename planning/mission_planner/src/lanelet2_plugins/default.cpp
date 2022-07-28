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

#include "default.hpp"

namespace mission_planner::lanelet2
{

void Default::Initialize(rclcpp::Node * node) { (void)node; }

bool Default::Ready() const { return true; }

MissionPlannerPlugin::HADMapRoute Default::Plan(const RoutePoints & points)
{
  (void)points;
  return HADMapRoute();
}

MissionPlannerPlugin::MarkerArray Default::Visualize(const HADMapRoute & route) const
{
  (void)route;
  return MarkerArray();
}

}  // namespace mission_planner::lanelet2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mission_planner::lanelet2::Default, mission_planner::MissionPlannerPlugin)
