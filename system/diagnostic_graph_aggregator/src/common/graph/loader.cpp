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

#include "loader.hpp"

#include "config.hpp"
#include "error.hpp"
#include "names.hpp"
#include "types.hpp"
#include "units.hpp"

#include <utility>

namespace diagnostic_graph_aggregator
{

GraphLoader::GraphLoader(const std::string & file)
{
  TreeLoader tree = TreeLoader::Load(file);
  FileConfig root = tree.construct();

  // Init array index to be able get it from unit itself.
  std::vector<UnitConfig *> diags;
  std::vector<UnitConfig *> nodes;
  for (const auto & config : root.units) {
    (config->type == unit_name::diag ? diags : nodes).push_back(config.get());
  }
  for (size_t i = 0; i < diags.size(); ++i) diags[i]->index = i;
  for (size_t i = 0; i < nodes.size(); ++i) nodes[i]->index = i;

  // Create link objects.
  std::unordered_map<LinkConfig *, UnitLink *> config_links;
  std::unordered_map<UnitConfig *, std::vector<UnitLink *>> parent_links;
  for (const auto & config : root.units) {
    parent_links[config.get()] = {};
  }
  for (const auto & config : root.links) {
    const auto link = links_.emplace_back(create_link()).get();
    config_links[config.get()] = link;
    parent_links[config->child].push_back(link);
  }

  // Create node objects.
  const auto links = GraphLinks{config_links, parent_links};
  std::unordered_map<UnitConfig *, BaseUnit *> config_units;
  for (const auto & config : diags) {
    const auto diag = diags_.emplace_back(create_diag(config, links)).get();
    config_units[config] = diag;
  }
  for (const auto & config : nodes) {
    const auto node = nodes_.emplace_back(create_node(config, links)).get();
    config_units[config] = node;
  }

  // Connect links and nodes;
  for (const auto & [config, link] : config_links) {
    const auto parent = config_units.at(config->parent);
    const auto child = config_units.at(config->child);
    link->initialize_object(parent, child);
  }

  // Init struct.
  for (auto & node : nodes_) node->initialize_struct();
  for (auto & diag : diags_) diag->initialize_struct();
  for (auto & link : links_) link->initialize_struct();

  // Init status that needs struct.
  for (auto & node : nodes_) node->initialize_status();
  for (auto & diag : diags_) diag->initialize_status();
  for (auto & link : links_) link->initialize_status();
}

std::vector<std::unique_ptr<UnitLink>> GraphLoader::release_links()
{
  return std::move(links_);
}

std::vector<std::unique_ptr<NodeUnit>> GraphLoader::release_nodes()
{
  return std::move(nodes_);
}

std::vector<std::unique_ptr<DiagUnit>> GraphLoader::release_diags()
{
  return std::move(diags_);
}

std::unique_ptr<UnitLink> GraphLoader::create_link()
{
  return std::make_unique<UnitLink>();
}

std::unique_ptr<DiagUnit> GraphLoader::create_diag(UnitConfig * config, const GraphLinks & links)
{
  if (config->type == unit_name::diag) {
    return std::make_unique<DiagUnit>(config, links);
  }
  throw UnknownUnitType(config->data.path(), config->type);
}

std::unique_ptr<NodeUnit> GraphLoader::create_node(UnitConfig * config, const GraphLinks & links)
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
  if (config->type == "warn-to-ok") {
    return std::make_unique<WarnToOkUnit>(config, links);
  }
  if (config->type == "warn-to-error") {
    return std::make_unique<WarnToErrorUnit>(config, links);
  }
  if (config->type == "ok") {
    return std::make_unique<OkUnit>(config, links);
  }
  if (config->type == "warn") {
    return std::make_unique<WarnUnit>(config, links);
  }
  if (config->type == "error") {
    return std::make_unique<ErrorUnit>(config, links);
  }
  if (config->type == "stale") {
    return std::make_unique<StaleUnit>(config, links);
  }
  throw UnknownUnitType(config->data.path(), config->type);
}

}  // namespace diagnostic_graph_aggregator
