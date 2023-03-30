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

#include "module.hpp"

#include <rclcpp/rclcpp.hpp>

#include <utility>

namespace behavior_velocity_planner::v2x_gate
{

SceneModule::SceneModule(const V2xGateData::ConstPtr data) { data_ = std::move(data); }

void SceneModule::plan(PathWithLaneId * path)
{
  (void)path;

  const auto logger = rclcpp::get_logger("behavior_velocity_planner.v2x_gate");
  RCLCPP_INFO_STREAM(logger, "scene module: " << data_->gate->id());
}

}  // namespace behavior_velocity_planner::v2x_gate
