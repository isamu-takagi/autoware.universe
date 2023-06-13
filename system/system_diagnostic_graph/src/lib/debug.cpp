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

#include "debug.hpp"

#include "graph.hpp"
#include "node.hpp"
#include "types.hpp"

#include <algorithm>
#include <iostream>
#include <unordered_map>
#include <vector>

namespace system_diagnostic_graph
{

const std::unordered_map<DiagnosticLevel, std::string> level_names = {
  {DiagnosticStatus::OK, "OK"},
  {DiagnosticStatus::WARN, "WARN"},
  {DiagnosticStatus::ERROR, "ERROR"},
  {DiagnosticStatus::STALE, "STALE"}};

void DiagGraph::debug()
{
  std::vector<DiagDebugData> lines;
  for (const auto & leaf : data_.leaf_list) {
    lines.push_back(leaf->debug());
  }
  for (const auto & unit : data_.unit_list) {
    lines.push_back(unit->debug());
  }

  std::array<size_t, diag_debug_size> widths = {};
  for (const auto & line : lines) {
    for (size_t i = 0; i < diag_debug_size; ++i) {
      widths[i] = std::max(widths[i], line[i].length());
    }
  }

  std::cout << "============================================================" << std::endl;
  for (const auto & line : lines) {
    for (size_t i = 0; i < diag_debug_size; ++i) {
      std::cout << "| " << std::left << std::setw(widths[i]) << line[i] << " ";
    }
    std::cout << "|" << std::endl;
  }
}

DiagDebugData DiagUnit::debug()
{
  return DiagDebugData{"unit", key_, "-----", "-----"};
}

DiagDebugData DiagLeaf::debug()
{
  return DiagDebugData{"diag", key_.first, key_.second, level_names.at(level_)};
}

}  // namespace system_diagnostic_graph
