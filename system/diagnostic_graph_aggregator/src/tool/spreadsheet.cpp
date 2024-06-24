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

#include "graph/graph.hpp"
#include "graph/units.hpp"

#include <iostream>

namespace diagnostic_graph_aggregator
{

enum class LinkType {
  Root,
  Node,
  Leaf,
};

struct TableRow
{
  const BaseUnit * unit;
  int depth;
  bool is_leaf;
};

struct Table
{
public:
  explicit Table(const std::string & path);
  std::vector<TableRow> rows_;

private:
  void flatten(const BaseUnit * unit, int depth, LinkType type);
  Graph graph_;
};

Table::Table(const std::string & path)
{
  graph_.create(path);

  for (const auto & unit : graph_.units()) {
    if (unit->parent_size() == 0 && unit->child_links().size() != 0) {
      flatten(unit, 0, LinkType::Root);
    }
  }
  for (const auto & unit : graph_.units()) {
    if (unit->parent_size() == 0 && unit->child_links().size() == 0) {
      flatten(unit, 0, LinkType::Root);
    }
  }
  for (const auto & unit : graph_.units()) {
    if (unit->parent_size() >= 2 && unit->child_links().size() != 0) {
      flatten(unit, 0, LinkType::Root);
    }
  }
}

void Table::flatten(const BaseUnit * unit, int depth, LinkType type)
{
  const auto is_root = (type == LinkType::Root);
  const auto is_node = (unit->parent_size() == 1);
  rows_.push_back(TableRow{unit, depth, false});

  if (is_root || is_node) {
    for (const auto link : unit->child_links()) {
      flatten(link->child(), depth + 1, LinkType::Node);
    }
  }
}

}  // namespace diagnostic_graph_aggregator

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: tree <path>" << std::endl;
    return 1;
  }

  auto table = diagnostic_graph_aggregator::Table(argv[1]);
  for (const auto & row : table.rows_) {
    std::cout << row.depth << " " << row.unit->path() << std::endl;
  }
}
