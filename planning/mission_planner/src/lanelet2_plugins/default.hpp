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

#ifndef LANELET2_PLUGINS__DEFAULT_HPP_
#define LANELET2_PLUGINS__DEFAULT_HPP_

#include <mission_planner/mission_planner_plugin.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mission_planner::lanelet2
{

class Default : public mission_planner::MissionPlannerPlugin
{
public:
  void Initialize(rclcpp::Node * node) override;
  bool Ready() const override;
  HADMapRoute Plan(const RoutePoints & points) override;
  MarkerArray Visualize(const HADMapRoute & route) const override;
};

}  // namespace mission_planner::lanelet2

#endif  // LANELET2_PLUGINS__DEFAULT_HPP_
