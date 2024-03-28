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

#ifndef DIAGNOSTIC_GRAPH_UTILS__STRUCTURE_HPP_
#define DIAGNOSTIC_GRAPH_UTILS__STRUCTURE_HPP_

#include <rclcpp/time.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <tier4_system_msgs/msg/diag_graph_status.hpp>
#include <tier4_system_msgs/msg/diag_graph_struct.hpp>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace diagnostic_graph_utils
{

struct DiagLink
{
  bool used;
};

struct DiagNode
{
  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
  std::string type;
  std::string name;
  std::vector<std::pair<DiagNode *, DiagLink *>> children;
  DiagnosticStatus status;
};

class DiagGraph
{
public:
  using DiagGraphStruct = tier4_system_msgs::msg::DiagGraphStruct;
  using DiagGraphStatus = tier4_system_msgs::msg::DiagGraphStatus;
  void update_struct(const DiagGraphStruct & msg);
  void update_status(const DiagGraphStatus & msg);
  const std::vector<const DiagNode *> & nodes() const { return units_; }

private:
  std::optional<rclcpp::Time> stamp_;
  std::vector<std::unique_ptr<DiagNode>> nodes_;
  std::vector<std::unique_ptr<DiagNode>> diags_;
  std::vector<std::unique_ptr<DiagLink>> links_;
  std::vector<const DiagNode *> units_;
};

}  // namespace diagnostic_graph_utils

#endif  // DIAGNOSTIC_GRAPH_UTILS__STRUCTURE_HPP_
