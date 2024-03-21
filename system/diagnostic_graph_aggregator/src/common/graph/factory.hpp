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

#ifndef COMMON__GRAPH__FACTORY_HPP_
#define COMMON__GRAPH__FACTORY_HPP_

#include "config.hpp"
#include "types.hpp"

#include <memory>
#include <unordered_map>
#include <vector>

namespace diagnostic_graph_aggregator
{

class LinkFactory
{
public:
  UnitLink * create(BaseUnit * parent, UnitConfig::SharedPtr config);
  void connect(BaseUnit * child, UnitConfig::SharedPtr config);
  std::vector<std::unique_ptr<UnitLink>> release_links();
  std::vector<UnitLink *> create(
    BaseUnit * parent, const std::vector<UnitConfig::SharedPtr> & configs);

private:
  std::unordered_multimap<UnitConfig::SharedPtr, std::unique_ptr<UnitLink>> links_;
};

class UnitFactory
{
public:
  BaseUnit * create(UnitConfig::SharedPtr config, LinkFactory & links);
  std::vector<std::unique_ptr<NodeUnit>> release_nodes();
  std::vector<std::unique_ptr<DiagUnit>> release_diags();

private:
  std::vector<std::unique_ptr<NodeUnit>> nodes_;
  std::vector<std::unique_ptr<DiagUnit>> diags_;
};

}  // namespace diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__FACTORY_HPP_
