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

UnitLink * LinkFactory::create(BaseUnit * parent, UnitConfig::SharedPtr config)
{
  const auto iter = links_.emplace(config, std::make_unique<UnitLink>());
  const auto link = iter->second.get();
  link->parent_ = parent;
  return link;
}

std::vector<UnitLink *> LinkFactory::create(
  BaseUnit * parent, const std::vector<UnitConfig::SharedPtr> & configs)
{
  std::vector<UnitLink *> result;
  for (const auto & config : configs) result.push_back(create(parent, config));
  return result;
}

void LinkFactory::connect(BaseUnit * child, UnitConfig::SharedPtr config)
{
  const auto range = links_.equal_range(config);
  for (auto iter = range.first; iter != range.second; ++iter) {
    iter->second->child_ = child;
  }
}

std::vector<std::unique_ptr<UnitLink>> LinkFactory::release_links()
{
  std::vector<std::unique_ptr<UnitLink>> result;
  for (auto & [config, link] : links_) result.push_back(std::move(link));
  return result;
}

std::unique_ptr<DiagUnit> create_diag(UnitConfig::SharedPtr config, LinkFactory & links)
{
  return std::make_unique<DiagUnit>(config, links);
}

std::unique_ptr<NodeUnit> create_node(UnitConfig::SharedPtr config, LinkFactory & links)
{
  if (config->type == "and") {
    return std::make_unique<MaxUnit>(config, links, false);
  }
  if (config->type == "short-circuit-and") {
    return std::make_unique<MaxUnit>(config, links, true);
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
    return std::make_unique<ConstUnit>(config, DiagnosticStatus::OK);
  }
  if (config->type == "warn") {
    return std::make_unique<ConstUnit>(config, DiagnosticStatus::WARN);
  }
  if (config->type == "error") {
    return std::make_unique<ConstUnit>(config, DiagnosticStatus::ERROR);
  }
  if (config->type == "stale") {
    return std::make_unique<ConstUnit>(config, DiagnosticStatus::STALE);
  }
  throw error<UnknownType>("unknown node type", config->type, config->data);
}

BaseUnit * UnitFactory::create(UnitConfig::SharedPtr config, LinkFactory & links)
{
  if (config->type == "diag") {
    return diags_.emplace_back(create_diag(config, links)).get();
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
