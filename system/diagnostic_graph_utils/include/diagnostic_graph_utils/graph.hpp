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

#ifndef DIAGNOSTIC_GRAPH_UTILS__GRAPH_HPP_
#define DIAGNOSTIC_GRAPH_UTILS__GRAPH_HPP_

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

class DiagUnit
{
public:
  using DiagLevel = diagnostic_msgs::msg::DiagnosticStatus::_level_type;
  virtual DiagLevel level() const = 0;
  virtual const std::string & type() const = 0;
  virtual const std::string & path() const = 0;
};

class DiagLink
{
  // Not supported now.
};

class DiagNode : public DiagUnit
{
public:
  using DiagNodeStruct = tier4_system_msgs::msg::DiagNodeStruct;
  using DiagNodeStatus = tier4_system_msgs::msg::DiagNodeStatus;
  explicit DiagNode(const DiagNodeStruct & msg) : struct_(msg) {}
  void update(const DiagNodeStatus & msg) { status_ = msg; }

  DiagLevel level() const override { return status_.level; }
  const std::string & type() const override { return struct_.type; }
  const std::string & path() const override { return struct_.path; }

private:
  DiagNodeStruct struct_;
  DiagNodeStatus status_;
};

class DiagLeaf : public DiagUnit
{
public:
  using DiagLeafStruct = tier4_system_msgs::msg::DiagLeafStruct;
  using DiagLeafStatus = tier4_system_msgs::msg::DiagLeafStatus;
  explicit DiagLeaf(const DiagLeafStruct & msg) : struct_(msg) {}
  void update(const DiagLeafStatus & msg) { status_ = msg; }
  const DiagLeafStatus & status() const { return status_; }

  DiagLevel level() const override { return status_.level; }
  const std::string & type() const override { return struct_.type; }
  const std::string & path() const override { return struct_.path; }

private:
  DiagLeafStruct struct_;
  DiagLeafStatus status_;
};

struct DiagGraph
{
public:
  using DiagGraphStruct = tier4_system_msgs::msg::DiagGraphStruct;
  using DiagGraphStatus = tier4_system_msgs::msg::DiagGraphStatus;
  using SharedPtr = std::shared_ptr<DiagGraph>;
  using ConstSharedPtr = std::shared_ptr<const DiagGraph>;
  void create(const DiagGraphStruct & msg);
  bool update(const DiagGraphStatus & msg);
  const auto & nodes() const { return nodes_; }
  const auto & diags() const { return diags_; }
  const auto & links() const { return links_; }

private:
  std::string name_;
  std::vector<std::unique_ptr<DiagNode>> nodes_;
  std::vector<std::unique_ptr<DiagLeaf>> diags_;
  std::vector<std::unique_ptr<DiagLink>> links_;
};

}  // namespace diagnostic_graph_utils

#endif  // DIAGNOSTIC_GRAPH_UTILS__GRAPH_HPP_
