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
#include <unordered_map>
#include <vector>

namespace diagnostic_graph_aggregator
{

struct Graph
{
  void create(const std::string & file);
  bool update(const rclcpp::Time & stamp, const DiagnosticStatus & status);
  DiagGraphStruct create_struct(const rclcpp::Time & stamp);
  DiagGraphStatus create_status(const rclcpp::Time & stamp);

  std::vector<std::unique_ptr<NodeUnit>> nodes_;
  std::vector<std::unique_ptr<DiagUnit>> diags_;
  std::vector<std::unique_ptr<UnitLink>> links_;
  std::vector<BaseUnit *> units_;
  std::unordered_map<std::string, DiagUnit *> names_;

  // For unique_ptr.
  Graph();
  ~Graph();
};

}  // namespace diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__GRAPH_HPP_
