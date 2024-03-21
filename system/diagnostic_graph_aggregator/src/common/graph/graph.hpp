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

#ifndef COMMON__GRAPH__GRAPH_HPP_
#define COMMON__GRAPH__GRAPH_HPP_

#include "types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace diagnostic_graph_aggregator
{

struct Graph
{
  static Graph load(const std::string & file);
  Graph() = default;
  Graph(const Graph &) = delete;
  Graph(Graph &&) = default;
  ~Graph();  // To delete unique_ptr.

  std::vector<std::unique_ptr<NodeUnit>> nodes;
  std::vector<std::unique_ptr<DiagUnit>> diags;
  std::vector<std::unique_ptr<UnitLink>> links;
  std::vector<BaseUnit *> units;
};

}  // namespace diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__GRAPH_HPP_
