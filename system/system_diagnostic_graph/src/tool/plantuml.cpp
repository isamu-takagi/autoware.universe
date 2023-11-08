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
#include "types.hpp"
#include "units.hpp"

#include <iostream>
#include <string>
#include <unordered_map>

namespace system_diagnostic_graph
{

struct UnitData
{
  std::string path;
  std::vector<UnitData *> children;
  std::vector<UnitData *> parents;
};

std::vector<std::unique_ptr<UnitData>> load_unit_data(const std::string & path)
{
  std::unordered_map<BaseUnit *, std::unique_ptr<UnitData>> mapping;
  Graph graph;
  graph.init(path);

  for (const auto & node : graph.nodes()) {
    auto data = std::make_unique<UnitData>();
    data->path = node->path();
    mapping[node] = std::move(data);
  }

  for (const auto & [node, data] : mapping) {
    for (const auto & link : node->children()) {
      const auto parent = data.get();
      const auto child = mapping.at(link).get();
      child->parents.push_back(parent);
      parent->children.push_back(child);
    }
  }

  std::vector<std::unique_ptr<UnitData>> result;
  for (auto & [node, data] : mapping) {
    result.push_back(std::move(data));
  }
  return result;
}

void dump_unit_card(const UnitData * unit, const std::string & suffix)
{
  const auto color = "#FFFFFF";
  std::cout << "card " << unit << suffix << " " << color << " [" << std::endl;
  std::cout << unit->path << std::endl;
  std::cout << "]" << std::endl;
}

void dump_unit_data(const std::vector<std::unique_ptr<UnitData>> & units)
{
  for (const auto & unit : units) {
    if (unit->parents.size() == 1) {
      dump_unit_card(unit.get(), "");
    } else {
      dump_unit_card(unit.get(), "_p");
      dump_unit_card(unit.get(), "_c");
    }
  }

  for (const auto & unit : units) {
    for (const auto & child : unit->children) {
      std::cout << unit.get() << " --> " << child << std::endl;
    }
  }
}

void plantuml(const std::string & path)
{
  dump_unit_data(load_unit_data(path));
}

}  // namespace system_diagnostic_graph

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: plantuml <path>" << std::endl;
    return 1;
  }
  system_diagnostic_graph::plantuml(argv[1]);
}
