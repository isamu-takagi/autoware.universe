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
  for (const auto & unit : graph.nodes) units.push_back(unit.get());
  for (const auto & unit : graph.diags) units.push_back(unit.get());

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
    throw error<GraphStructure>("detect graph circulation");
  }

  // Reverse the result to process from leaf node.
  return std::vector<BaseUnit *>(result.rbegin(), result.rend());
}

Graph::~Graph()
{
  // To delete unique_ptr.
}

void Graph::load(const std::string & file)
{
  UnitFactory unit_factory;
  LinkFactory link_factory;
  std::unordered_map<UnitConfig::SharedPtr, BaseUnit *> unit_mapping;

  for (const auto & config : load_root_config(file).nodes) {
    unit_mapping[config] = unit_factory.create(config, link_factory);
  }
  for (const auto & [config, unit] : unit_mapping) {
    link_factory.connect(unit, config);
  }

  nodes = unit_factory.release_nodes();
  diags = unit_factory.release_diags();
  links = link_factory.release_links();
  units = topological_sort(*this);
}

/*
void Graph::init(const std::string & file)
{
  // Sort units in topological order for update dependencies.
  nodes = topological_sort(std::move(nodes));

  // List diag nodes that have diag name.
  for (const auto & node : nodes) {
    const auto diag = dynamic_cast<DiagTempUnit *>(node.get());
    if (diag) {
      diags_[diag->name()] = diag;
    }
  }

  // List unit nodes that have path name.
  for (const auto & node : nodes) {
    if (!node->path().empty()) {
      units_.push_back(node.get());
    }
  }

  // Set unit index.
  for (size_t index = 0; index < units_.size(); ++index) {
    units_[index]->set_index(index);
  }

  nodes_ = std::move(nodes);
}

void Graph::callback(const rclcpp::Time & stamp, const DiagnosticArray & array)
{
  for (const auto & status : array.status) {
    const auto iter = diags_.find(status.name);
    if (iter != diags_.end()) {
      iter->second->callback(stamp, status);
    } else {
      unknowns_[status.name] = status.level;
    }
  }
}

DiagnosticGraph Graph::report(const rclcpp::Time & stamp)
{
  for (const auto & node : nodes_) {
    node->update(stamp);
  }

  DiagnosticGraph message;
  message.stamp = stamp;
  message.nodes.reserve(units_.size());
  for (const auto & node : units_) {
    const auto report = node->report();
    DiagnosticNode temp;
    temp.status.name = node->path();
    temp.status.level = report.level;
    for (const auto & [ref, used] : report.links) {
      DiagnosticLink link;
      link.index = ref->index();
      link.used = used;
      temp.links.push_back(link);
    }
    message.nodes.push_back(temp);
  }
  return message;
}

std::vector<BaseUnit *> Graph::nodes() const
{
  std::vector<BaseTempUnit *> result;
  result.reserve(nodes_.size());
  for (const auto & node : nodes_) {
    result.push_back(node.get());
  }
  return result;
}

*/

}  // namespace diagnostic_graph_aggregator
