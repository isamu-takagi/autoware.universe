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

#ifndef MODULE_HPP_
#define MODULE_HPP_

#include <behavior_velocity_planner/scene_module_plugin.hpp>
#include <lanelet2_extension/regulatory_elements/v2x_gate.hpp>

#include <memory>
#include <unordered_map>

namespace behavior_velocity_planner::v2x_gate
{

struct V2xGateData
{
  using ConstPtr = std::shared_ptr<V2xGateData>;
  lanelet::autoware::V2xGate::ConstPtr gate;
  std::unordered_map<lanelet::Id, lanelet::ConstLineString3d> acquire_lines;
  std::unordered_map<lanelet::Id, lanelet::ConstLineString3d> release_lines;
};

struct FrameData
{
};

class SceneModule : public SceneModulePlugin
{
public:
  using SharedPtr = std::shared_ptr<SceneModule>;
  explicit SceneModule(const V2xGateData::ConstPtr data);
  void plan(PathWithLaneId * path, const FrameData & frame);

private:
  V2xGateData::ConstPtr data_;
};

}  // namespace behavior_velocity_planner::v2x_gate

#endif  // MODULE_HPP_
