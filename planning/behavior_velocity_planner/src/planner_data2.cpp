// Copyright 2023 The Autoware Contributors
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

#include <behavior_velocity_planner/planner_data.hpp>
#include <behavior_velocity_planner/planner_data/common.hpp>
#include <behavior_velocity_planner/planner_data2.hpp>

namespace behavior_velocity_planner
{

PlannerData2::PlannerData2(const PlannerData & data)
{
  common = std::make_unique<PlannerDataCommon>();

  common->route_handler = data.route_handler_;
  common->current_odometry = data.current_odometry;
}

PlannerData2::~PlannerData2()
{
  // For unique_ptr destructor.
}

}  // namespace behavior_velocity_planner
