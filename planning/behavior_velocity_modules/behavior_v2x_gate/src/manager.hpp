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

#ifndef MANAGER_HPP_
#define MANAGER_HPP_

#include "module.hpp"
#include "status.hpp"

#include <behavior_velocity_planner/scene_manager_plugin.hpp>

#include <memory>  // TODO(Takagi, Isamu): Replace with PlannerData2
#include <unordered_map>

namespace behavior_velocity_planner::v2x_gate
{

class SceneManager : public SceneManagerPlugin
{
public:
  void init(rclcpp::Node * node);
  void plan(PathWithLaneId * path) override;
  void update(const PlannerData2::ConstSharedPtr & data, const PathWithLaneId & path) override;

  boost::optional<int> getFirstStopPathPointIndex() override;
  const char * getModuleName() override;

private:
  rclcpp::Node * node_;
  std::unordered_map<lanelet::Id, SceneModule::SharedPtr> scenes_;
  std::unique_ptr<LockServer> server_;
  PlannerData2::ConstSharedPtr data_;
};

}  // namespace behavior_velocity_planner::v2x_gate

#endif  // MANAGER_HPP_
