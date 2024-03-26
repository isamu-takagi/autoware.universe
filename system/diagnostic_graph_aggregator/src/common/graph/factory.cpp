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

#include "factory.hpp"

#include "error.hpp"
#include "units.hpp"

#include <utility>

namespace diagnostic_graph_aggregator
{

UnitLink * LinkFactory::create(NodeUnit * parent, UnitConfigItem config)
{
  const auto link = links_.emplace_back(std::make_unique<UnitLink>()).get();
  link->set_parent(parent);
  mapping_.emplace(config, link);
  return link;
}

std::vector<UnitLink *> LinkFactory::create(NodeUnit * parent, const UnitConfigList & configs)
{
  std::vector<UnitLink *> result;
  for (const auto & config : configs) {
    result.push_back(create(parent, config));
  }
  return result;
}

std::vector<UnitLink *> LinkFactory::connect(BaseUnit * child, UnitConfigItem config)
{
  const auto range = mapping_.equal_range(config);
  std::vector<UnitLink *> result;
  for (auto iter = range.first; iter != range.second; ++iter) {
    const auto link = iter->second;
    link->set_child(child);
    result.push_back(link);
  }
  return result;
}

std::vector<std::unique_ptr<UnitLink>> LinkFactory::release_links()
{
  return std::move(links_);
}

std::unique_ptr<DiagUnit> create_diag(UnitConfigItem config)
{
  return std::make_unique<DiagUnit>(config);
}

std::unique_ptr<NodeUnit> create_node(UnitConfigItem config, LinkFactory & links)
{
  if (config->type == "and") {
    return std::make_unique<MaxUnit>(config, links);
  }
  if (config->type == "short-circuit-and") {
    return std::make_unique<ShortCircuitMaxUnit>(config, links);
  }
  if (config->type == "or") {
    return std::make_unique<MinUnit>(config, links);
  }
  /*
  if (config->type == "warn-to-ok") {
    return std::make_unique<RemapUnit>(config->path, DiagnosticStatus::OK);
  }
  if (config->type == "warn-to-error") {
    return std::make_unique<RemapUnit>(config->path, DiagnosticStatus::ERROR);
  }
  */
  if (config->type == "ok") {
    return std::make_unique<OkUnit>(config);
  }
  if (config->type == "warn") {
    return std::make_unique<WarnUnit>(config);
  }
  if (config->type == "error") {
    return std::make_unique<ErrorUnit>(config);
  }
  if (config->type == "stale") {
    return std::make_unique<StaleUnit>(config);
  }
  throw UnknownUnitType(config->data.path(), config->type);
}

BaseUnit * UnitFactory::create(UnitConfigItem config, LinkFactory & links)
{
  if (config->type == "diag") {
    return diags_.emplace_back(create_diag(config)).get();
  } else {
    return nodes_.emplace_back(create_node(config, links)).get();
  }
}

std::vector<std::unique_ptr<NodeUnit>> UnitFactory::release_nodes()
{
  return std::move(nodes_);
}

std::vector<std::unique_ptr<DiagUnit>> UnitFactory::release_diags()
{
  return std::move(diags_);
}

}  // namespace diagnostic_graph_aggregator
