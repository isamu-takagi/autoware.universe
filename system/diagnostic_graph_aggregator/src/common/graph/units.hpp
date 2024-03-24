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

#include <optional>
#include <string>
#include <vector>

namespace diagnostic_graph_aggregator
{

class VectorElement
{
public:
  size_t get_index() const { return index_; }
  void set_index(size_t index) { index_ = index; }

private:
  size_t index_;
};

class UnitLink : private VectorElement
{
public:
  DiagLinkStruct get_struct() const { return struct_; }
  DiagLinkStatus get_status() const { return status_; }
  NodeUnit * get_parent() const { return parent_; }
  BaseUnit * get_child() const { return child_; }
  void set_parent(NodeUnit * parent) { parent_ = parent; }
  void set_child(BaseUnit * child) { child_ = child; }
  void initialize_struct();  // This function must be called after setting the index.
  void initialize_status();  // This function must be called after setting the struct.
  using VectorElement::get_index;
  using VectorElement::set_index;

private:
  NodeUnit * parent_;
  BaseUnit * child_;
  DiagLinkStruct struct_;
  DiagLinkStatus status_;
};

class BaseUnit : private VectorElement
{
public:
  virtual ~BaseUnit() = default;
  virtual DiagnosticLevel get_level() const = 0;
  virtual std::string get_path() const = 0;
  virtual std::string get_type() const = 0;
  virtual std::vector<UnitLink *> get_child_links() const = 0;
  virtual std::vector<BaseUnit *> get_child_units() const;
  virtual bool is_leaf() const = 0;
  virtual void initialize_struct();  // This function must be called after setting the index.
  virtual void initialize_status();  // This function must be called after setting the struct.
  void set_parent_links(const std::vector<UnitLink *> & parents) { parents_ = parents; }
  using VectorElement::get_index;
  using VectorElement::set_index;

protected:
  bool update();

private:
  virtual void update_status() = 0;  // Type dependent part of the update function.

  DiagnosticLevel prev_level_;
  std::vector<UnitLink *> parents_;
};

class NodeUnit : public BaseUnit
{
public:
  explicit NodeUnit(const UnitConfig::SharedPtr & config);
  DiagNodeStruct get_struct() const { return struct_; }
  DiagNodeStatus get_status() const { return status_; }
  DiagnosticLevel get_level() const override { return status_.level; }
  std::string get_path() const override { return struct_.path; }
  bool is_leaf() const override { return false; }
  void initialize_struct() override;  // This function must be called after setting the index.

protected:
  DiagNodeStruct struct_;
  DiagNodeStatus status_;
};

class LeafUnit : public BaseUnit
{
public:
  DiagLeafStruct get_struct() const { return struct_; }
  DiagLeafStatus get_status() const { return status_; }
  DiagnosticLevel get_level() const override { return status_.level; }
  std::string get_name() const { return struct_.name; }
  std::string get_path() const override { return struct_.path; }
  bool is_leaf() const override { return true; }

protected:
  DiagLeafStruct struct_;
  DiagLeafStatus status_;
};

class DiagUnit : public LeafUnit
{
public:
  explicit DiagUnit(const UnitConfig::SharedPtr & config);
  std::string get_type() const override { return "diag"; }
  std::vector<UnitLink *> get_child_links() const override { return {}; }
  bool on_time(const rclcpp::Time & stamp);
  bool on_diag(const rclcpp::Time & stamp, const DiagnosticStatus & status);

private:
  void update_status() override;
  double timeout_;
  std::optional<rclcpp::Time> last_updated_time_;
};

class MaxUnit : public NodeUnit
{
public:
  MaxUnit(const UnitConfig::SharedPtr & config, LinkFactory & links, bool short_circuit);
  std::string get_type() const override { return short_circuit_ ? "short-circuit-and" : "and"; }
  std::vector<UnitLink *> get_child_links() const override { return links_; }

private:
  void update_status() override;
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
  void update_status() override;
  std::vector<UnitLink *> links_;
};

class ConstUnit : public NodeUnit
{
public:
  ConstUnit(const UnitConfig::SharedPtr & config, DiagnosticLevel level);
  std::string get_type() const override { return "const"; }
  std::vector<UnitLink *> get_child_links() const override { return {}; }

private:
  void update_status() override;
};

}  // namespace diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__UNITS_HPP_
