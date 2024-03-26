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

#include "graph.hpp"

#include "config.hpp"
#include "error.hpp"
#include "factory.hpp"
#include "units.hpp"

#include <deque>
#include <unordered_map>

namespace diagnostic_graph_aggregator
{

std::vector<BaseUnit *> topological_sort(const Graph & graph)
{
  std::unordered_map<BaseUnit *, int> degrees;
  std::deque<BaseUnit *> units;
  std::deque<BaseUnit *> result;
  std::deque<BaseUnit *> buffer;

  // Create a list of raw pointer units.
  for (const auto & unit : graph.nodes()) units.push_back(unit.get());
  for (const auto & unit : graph.diags()) units.push_back(unit.get());

  // Count degrees of each unit.
  for (const auto & unit : units) {
    for (const auto & child : unit->get_child_units()) ++degrees[child];
  }

  // Find initial units that are zero degrees.
  for (const auto & unit : units) {
    if (degrees[unit] == 0) buffer.push_back(unit);
  }

  // Sort by topological order.
  while (!buffer.empty()) {
    const auto unit = buffer.front();
    buffer.pop_front();
    for (const auto & child : unit->get_child_units()) {
      if (--degrees[child] == 0) {
        buffer.push_back(child);
      }
    }
    result.push_back(unit);
  }

  // Detect circulation because the result does not include the nodes on the loop.
  if (result.size() != units.size()) {
    throw GraphStructure("detect graph circulation");
  }

  // Reverse the result to process from leaf node.
  return std::vector<BaseUnit *>(result.rbegin(), result.rend());
}

void Graph::create(const std::string & file)
{
  UnitFactory unit_factory;
  LinkFactory link_factory;
  std::unordered_map<UnitConfigItem, BaseUnit *> unit_mapping;

  TreeLoader tree = TreeLoader::Load(file);
  FileConfig root = tree.flatten();

  // Create units and links.
  for (const auto & config : root.units) {
    unit_mapping[config] = unit_factory.create(config, link_factory);
  }
  for (const auto & [config, unit] : unit_mapping) {
    unit->set_parent_links(link_factory.connect(unit, config));
  }

  // Move units and links.
  nodes_ = unit_factory.release_nodes();
  diags_ = unit_factory.release_diags();
  links_ = link_factory.release_links();
  units_ = topological_sort(*this);
  for (const auto & diag : diags_) names_[diag->get_name()] = diag.get();

  // Init array index.
  for (size_t i = 0; i < nodes_.size(); ++i) nodes_[i]->set_index(i);
  for (size_t i = 0; i < diags_.size(); ++i) diags_[i]->set_index(i);
  for (size_t i = 0; i < links_.size(); ++i) links_[i]->set_index(i);

  // Init struct that needs array index.
  for (auto & unit : units_) unit->initialize_struct();
  for (auto & link : links_) link->initialize_struct();

  // Init status that needs struct.
  for (auto & unit : units_) unit->initialize_status();
  for (auto & link : links_) link->initialize_status();
}

void Graph::update(const rclcpp::Time & stamp)
{
  for (const auto & diag : diags_) diag->on_time(stamp);
}

bool Graph::update(const rclcpp::Time & stamp, const DiagnosticStatus & status)
{
  const auto iter = names_.find(status.name);
  if (iter == names_.end()) return false;
  iter->second->on_diag(stamp, status);
  return true;
}

DiagGraphStruct Graph::create_struct(const rclcpp::Time & stamp) const
{
  DiagGraphStruct msg;
  msg.stamp = stamp;
  for (const auto & node : nodes_) msg.nodes.push_back(node->get_struct());
  for (const auto & diag : diags_) msg.diags.push_back(diag->get_struct());
  for (const auto & link : links_) msg.links.push_back(link->get_struct());
  return msg;
}

DiagGraphStatus Graph::create_status(const rclcpp::Time & stamp) const
{
  DiagGraphStatus msg;
  msg.stamp = stamp;
  for (const auto & node : nodes_) msg.nodes.push_back(node->get_status());
  for (const auto & diag : diags_) msg.diags.push_back(diag->get_status());
  for (const auto & link : links_) msg.links.push_back(link->get_status());
  return msg;
}

// For unique_ptr members.
Graph::Graph() = default;
Graph::~Graph() = default;

}  // namespace diagnostic_graph_aggregator
