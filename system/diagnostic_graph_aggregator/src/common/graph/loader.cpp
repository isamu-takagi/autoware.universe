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
#include "types/names.hpp"
#include "units.hpp"

#include <utility>

namespace diagnostic_graph_aggregator
{

class LinkerImpl : public Linker
{
public:
  std::vector<UnitLink *> take_parents(BaseUnit * unit) override;
  std::vector<UnitLink *> take_child_list(BaseUnit * unit) override;
  UnitLink * take_child_item(BaseUnit * unit) override;

  std::unordered_map<BaseUnit *, std::vector<UnitLink *>> parent_links_;
  std::unordered_map<BaseUnit *, std::vector<UnitLink *>> child_links_;
};

std::vector<UnitLink *> LinkerImpl::take_parents(BaseUnit * unit)
{
  return parent_links_[unit];
}

std::vector<UnitLink *> LinkerImpl::take_child_list(BaseUnit * unit)
{
  return child_links_[unit];
}

UnitLink * LinkerImpl::take_child_item(BaseUnit * unit)
{
  const auto links = take_child_list(unit);
  if (links.size() != 1) {
    throw std::logic_error(unit->get_type() + " unit should have only one child");
  }
  return links.at(0);
}

GraphLoader::GraphLoader(const std::string & file)
{
  TreeLoader tree = TreeLoader::Load(file);
  FileConfig root = tree.flatten();

  // Create units. The links will be set later.
  for (const auto & config : root.units) {
    const auto unit = create_unit(config.get());
    config_to_unit_[config.get()] = unit;
  }

  // Create links. Use a mapping from config to unit.
  LinkerImpl linker;
  for (const auto & config : root.links) {
    const auto link = create_link(config.get());
    linker.parent_links_[link->get_child()].push_back(link);
    linker.child_links_[link->get_parent()].push_back(link);
  }

  // Set parent and child links to units.
  for (auto & node : nodes_) node->initialize_parents(linker);
  for (auto & diag : diags_) diag->initialize_parents(linker);
  for (auto & node : nodes_) node->initialize_children(linker);
  for (auto & diag : diags_) diag->initialize_children(linker);

  // Init array index.
  for (size_t i = 0; i < nodes_.size(); ++i) nodes_[i]->set_index(i);
  for (size_t i = 0; i < diags_.size(); ++i) diags_[i]->set_index(i);
  for (size_t i = 0; i < links_.size(); ++i) links_[i]->set_index(i);

  // Init struct that needs array index.
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

BaseUnit * GraphLoader::create_unit(UnitConfigItem config)
{
  if (config->type == unit_name::diag) {
    return diags_.emplace_back(create_diag(config)).get();
  } else {
    return nodes_.emplace_back(create_node(config)).get();
  }
}

UnitLink * GraphLoader::create_link(LinkConfigItem config)
{
  const auto parent = config_to_unit_.at(config->parent);
  const auto child = config_to_unit_.at(config->child);
  return links_.emplace_back(std::make_unique<UnitLink>(parent, child)).get();
}

std::unique_ptr<DiagUnit> GraphLoader::create_diag(UnitConfigItem config)
{
  return std::make_unique<DiagUnit>(config);
}

std::unique_ptr<NodeUnit> GraphLoader::create_node(UnitConfigItem config)
{
  if (config->type == "and") {
    return std::make_unique<MaxUnit>(config);
  }
  if (config->type == "short-circuit-and") {
    return std::make_unique<ShortCircuitMaxUnit>(config);
  }
  if (config->type == "or") {
    return std::make_unique<MinUnit>(config);
  }
  if (config->type == "warn-to-ok") {
    return std::make_unique<WarnToOkUnit>(config);
  }
  if (config->type == "warn-to-error") {
    return std::make_unique<WarnToErrorUnit>(config);
  }
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

}  // namespace diagnostic_graph_aggregator
