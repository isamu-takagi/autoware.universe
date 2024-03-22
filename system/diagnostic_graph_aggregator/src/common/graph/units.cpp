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

#include "units.hpp"

#include "error.hpp"
#include "factory.hpp"

namespace diagnostic_graph_aggregator
{

DiagLinkStruct UnitLink::get_struct() const
{
  DiagLinkStruct msg;
  msg.parent = parent_->get_index();
  msg.child = child_->get_index();
  msg.is_leaf = child_->is_leaf();
  return msg;
}

DiagLinkStatus UnitLink::get_status() const
{
  DiagLinkStatus msg;
  return msg;
}

std::vector<BaseUnit *> BaseUnit::get_child_units() const
{
  std::vector<BaseUnit *> result;
  for (const auto & link : get_child_links()) result.push_back(link->get_child());
  return result;
}

DiagUnit::DiagUnit(const UnitConfig::SharedPtr & config)
{
  struct_.path = config->path;
}

void DiagUnit::on_diag(const rclcpp::Time & stamp, const DiagnosticStatus & status)
{
  last_updated_time_ = stamp;
  status_.level = status.level;
  status_.message = status.message;
  status_.hardware_id = status.hardware_id;
  status_.values = status.values;
}

MaxUnit::MaxUnit(const UnitConfig::SharedPtr & config, LinkFactory & links, bool short_circuit)
{
  short_circuit_ = short_circuit;

  struct_.path = config->path;
  links_ = links.create(this, config->children);
}

MinUnit::MinUnit(const UnitConfig::SharedPtr & config, LinkFactory & links)
{
  struct_.path = config->path;
  links_ = links.create(this, config->children);
}

ConstUnit::ConstUnit(const UnitConfig::SharedPtr & config, DiagnosticLevel level)
{
  struct_.path = config->path;
  status_.level = level;
}

/*
using LinkList = std::vector<std::pair<const BaseTempUnit *, bool>>;

void merge(LinkList & a, const LinkList & b, bool uses)
{
  for (const auto & [node, used] : b) {
    a.push_back(std::make_pair(node, used && uses));
  }
}

BaseTempUnit::BaseTempUnit(const std::string & path) : path_(path)
{
  index_ = 0;
  level_ = DiagnosticStatus::OK;
}

BaseTempUnit::NodeData BaseTempUnit::status() const
{
  if (path_.empty()) {
    return {level_, links_};
  } else {
    return {level_, {std::make_pair(this, true)}};
  }
}

BaseTempUnit::NodeData BaseTempUnit::report() const
{
  return {level_, links_};
}

void DiagTempUnit::init(const UnitConfig::SharedPtr & config, const NodeDict &)
{
  name_ = config->data.take_text("diag");
  timeout_ = config->data.take<double>("timeout", 1.0);
}

void DiagTempUnit::update(const rclcpp::Time & stamp)
{
  if (diagnostics_) {
    const auto updated = diagnostics_.value().first;
    const auto elapsed = (stamp - updated).seconds();
    if (timeout_ < elapsed) {
      diagnostics_ = std::nullopt;
    }
  }

  if (diagnostics_) {
    level_ = diagnostics_.value().second.level;
  } else {
    level_ = DiagnosticStatus::STALE;
  }
}

void AndUnit::update(const rclcpp::Time &)
{
  if (children_.empty()) {
    return;
  }

  bool uses = true;
  level_ = DiagnosticStatus::OK;
  links_ = LinkList();

  for (const auto & child : children_) {
    const auto status = child->status();
    level_ = std::max(level_, status.level);
    merge(links_, status.links, uses);
    if (short_circuit_ && level_ != DiagnosticStatus::OK) {
      uses = false;
    }
  }
  level_ = std::min(level_, DiagnosticStatus::ERROR);
}

void OrUnit::update(const rclcpp::Time &)
{
  if (children_.empty()) {
    return;
  }

  level_ = DiagnosticStatus::ERROR;
  links_ = LinkList();

  for (const auto & child : children_) {
    const auto status = child->status();
    level_ = std::min(level_, status.level);
    merge(links_, status.links, true);
  }
  level_ = std::min(level_, DiagnosticStatus::ERROR);
}

RemapUnit::RemapUnit(const std::string & path, DiagnosticLevel remap_warn) : BaseTempUnit(path)
{
  remap_warn_ = remap_warn;
}

void RemapUnit::init(const UnitConfig::SharedPtr & config, const NodeDict & dict)
{
  if (config->children.size() != 1) {
    throw error<InvalidValue>("list size must be 1", config->data);
  }
  children_ = resolve(dict, config->children);
}

void RemapUnit::update(const rclcpp::Time &)
{
  const auto status = children_.front()->status();
  level_ = status.level;
  links_ = status.links;

  if (level_ == DiagnosticStatus::WARN) level_ = remap_warn_;
}

DebugUnit::DebugUnit(const std::string & path, DiagnosticLevel level) : BaseTempUnit(path)
{
  level_ = level;  // overwrite
}

*/

}  // namespace diagnostic_graph_aggregator
