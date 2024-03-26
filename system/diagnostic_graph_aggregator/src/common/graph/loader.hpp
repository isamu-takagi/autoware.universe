// Copyright 2024 The Autoware Contributors
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

#ifndef COMMON__GRAPH__LOADER_HPP_
#define COMMON__GRAPH__LOADER_HPP_

#include "types/config.hpp"
#include "types/units.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace diagnostic_graph_aggregator
{

class GraphLoader
{
public:
  explicit GraphLoader(const std::string & file);
  std::vector<std::unique_ptr<NodeUnit>> release_nodes();
  std::vector<std::unique_ptr<DiagUnit>> release_diags();
  std::vector<std::unique_ptr<UnitLink>> release_links();

private:
  BaseUnit * create_unit(UnitConfigItem config);
  UnitLink * create_link(LinkConfigItem config);
  std::unique_ptr<DiagUnit> create_diag(UnitConfigItem config);
  std::unique_ptr<NodeUnit> create_node(UnitConfigItem config);

  // Note: keep correspondence between links and unit children order.
  std::vector<std::unique_ptr<NodeUnit>> nodes_;
  std::vector<std::unique_ptr<DiagUnit>> diags_;
  std::vector<std::unique_ptr<UnitLink>> links_;
  std::unordered_map<UnitConfigItem, BaseUnit *> config_to_unit_;
};

}  // namespace diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__LOADER_HPP_
