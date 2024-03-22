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

#ifndef COMMON__GRAPH__UNITS_HPP_
#define COMMON__GRAPH__UNITS_HPP_

#include "config.hpp"
#include "types.hpp"

#include <rclcpp/time.hpp>

#include <string>
#include <vector>

namespace diagnostic_graph_aggregator
{

class UnitLink
{
public:
  DiagLinkStruct get_struct() const;
  DiagLinkStatus get_status() const;
  BaseUnit * get_parent() const { return parent_; }
  BaseUnit * get_child() const { return child_; }

private:
  friend LinkFactory;
  BaseUnit * parent_;  // parent
  BaseUnit * child_;   // child
};

class BaseUnit
{
public:
  virtual DiagnosticLevel get_level() const = 0;
  virtual std::string get_path() const = 0;
  virtual std::string get_type() const = 0;
  virtual std::vector<UnitLink *> get_child_links() const = 0;
  virtual std::vector<BaseUnit *> get_child_units() const;
  virtual bool is_leaf() const = 0;

  size_t get_index() const { return index_; }
  void set_index(size_t index) { index_ = index; }

private:
  size_t index_;
};

class NodeUnit : public BaseUnit
{
public:
  DiagNodeStruct get_struct() const { return struct_; }
  DiagNodeStatus get_status() const { return status_; }
  DiagnosticLevel get_level() const override { return status_.level; }
  std::string get_path() const override { return struct_.path; }
  std::string get_type() const override { return "node"; }  // DEBUG
  bool is_leaf() const override { return false; }

protected:
  DiagNodeStruct struct_;
  DiagNodeStatus status_;
};

class DiagUnit : public BaseUnit
{
public:
  explicit DiagUnit(const UnitConfig::SharedPtr & config);
  DiagLeafStruct get_struct() const { return struct_; }
  DiagLeafStatus get_status() const { return status_; }
  DiagnosticLevel get_level() const override { return status_.level; }
  std::string get_name() const { return struct_.name; }
  std::string get_path() const override { return struct_.path; }
  std::string get_type() const override { return "diag"; }
  std::vector<UnitLink *> get_child_links() const override { return {}; }
  bool is_leaf() const override { return true; }
  void on_diag(const rclcpp::Time & stamp, const DiagnosticStatus & status);

private:
  rclcpp::Time last_updated_time_;
  double timeout_;
  DiagLeafStruct struct_;
  DiagLeafStatus status_;
};

class MaxUnit : public NodeUnit
{
public:
  MaxUnit(const UnitConfig::SharedPtr & config, LinkFactory & links, bool short_circuit);
  std::string get_type() const override { return short_circuit_ ? "short-circuit-and" : "and"; }
  std::vector<UnitLink *> get_child_links() const override { return links_; }

private:
  bool short_circuit_;
  std::vector<UnitLink *> links_;
};

class MinUnit : public NodeUnit
{
public:
  MinUnit(const UnitConfig::SharedPtr & config, LinkFactory & links);
  std::string get_type() const override { return "or"; }
  std::vector<UnitLink *> get_child_links() const override { return links_; }

private:
  std::vector<UnitLink *> links_;
};

class ConstUnit : public NodeUnit
{
public:
  ConstUnit(const UnitConfig::SharedPtr & config, DiagnosticLevel level);
  std::string get_type() const override { return "const"; }
  std::vector<UnitLink *> get_child_links() const override { return {}; }
};

/*
class BaseTempUnit
{
public:
  struct NodeDict
  {
    std::unordered_map<UnitConfig::SharedPtr, BaseTempUnit *> configs;
    std::unordered_map<std::string, BaseTempUnit *> paths;
  };
  struct NodeData
  {
    DiagnosticLevel level;
    std::vector<std::pair<const BaseTempUnit *, bool>> links;
  };

  explicit BaseTempUnit(const std::string & path);
  virtual ~BaseTempUnit() = default;
  virtual void init(const UnitConfig::SharedPtr & config, const NodeDict & dict) = 0;
  virtual void update(const rclcpp::Time & stamp) = 0;

  NodeData status() const;
  NodeData report() const;
  DiagnosticLevel level() const { return level_; }

  size_t index() const { return index_; }
  void set_index(const size_t index) { index_ = index; }

protected:
  DiagnosticLevel level_;
  std::string path_;
  std::vector<BaseTempUnit *> children_;
  std::vector<std::pair<const BaseTempUnit *, bool>> links_;

private:
  size_t index_;
};

*/

}  // namespace diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__UNITS_HPP_
