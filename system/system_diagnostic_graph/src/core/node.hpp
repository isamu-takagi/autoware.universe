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

#ifndef CORE__NODE_HPP_
#define CORE__NODE_HPP_

#include "config.hpp"
#include "debug.hpp"
#include "expr.hpp"
#include "types.hpp"

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace system_diagnostic_graph
{

class GraphData;

class BaseNode
{
public:
  virtual ~BaseNode() = default;
  virtual void update() = 0;
  virtual DiagnosticNode report() const = 0;
  virtual DiagDebugData debug() const = 0;
  virtual std::vector<BaseNode *> links() const = 0;
  virtual std::string name() const = 0;

  DiagnosticLevel level() const { return level_; }
  size_t index() const { return index_; }
  void set_index(const size_t index) { index_ = index; }

protected:
  size_t index_ = 0;
  DiagnosticLevel level_;
};

class UnitNode : public BaseNode
{
public:
  using KeyType = std::string;
  explicit UnitNode(const KeyType & key);

  DiagnosticNode report() const override;
  DiagDebugData debug() const override;
  void update() override;
  void create(GraphData & graph, const NodeConfig & config);

  std::vector<BaseNode *> links() const override { return links_; }
  std::string name() const override { return key_; }

private:
  const KeyType key_;
  std::vector<BaseNode *> links_;
  std::unique_ptr<BaseExpr> expr_;
};

class DiagNode : public BaseNode
{
public:
  using KeyType = std::pair<std::string, std::string>;
  explicit DiagNode(const KeyType & key);

  DiagnosticNode report() const override;
  DiagDebugData debug() const override;
  void update() override;
  void callback(const DiagnosticStatus & status);

  std::vector<BaseNode *> links() const override { return {}; }
  std::string name() const override { return key_.first; }

private:
  const KeyType key_;
  DiagnosticStatus status_;
};

struct GraphData
{
  std::vector<std::unique_ptr<BaseNode>> nodes;
  std::map<UnitNode::KeyType, UnitNode *> units;
  std::map<DiagNode::KeyType, DiagNode *> diags;
};

}  // namespace system_diagnostic_graph

#endif  // CORE__NODE_HPP_
