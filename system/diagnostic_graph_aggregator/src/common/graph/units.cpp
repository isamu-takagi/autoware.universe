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

#include "config.hpp"
#include "error.hpp"
#include "loader.hpp"

#include <algorithm>

namespace diagnostic_graph_aggregator
{

std::vector<UnitLink *> children(const GraphLinks & links, std::vector<LinkConfig *> configs)
{
  std::vector<UnitLink *> result;
  for (const auto & config : configs) result.push_back(links.config_links.at(config));
  return result;
}

void UnitLink::initialize_object(BaseUnit * parent, BaseUnit * child)
{
  parent_ = parent;
  child_ = child;
}

void UnitLink::initialize_struct()
{
  struct_.parent = parent_->get_index();
  struct_.child = child_->get_index();
  struct_.is_leaf = child_->is_leaf();
}

void UnitLink::initialize_status()
{
  // Do nothing.
}

BaseUnit::BaseUnit(const UnitConfig * config, const GraphLinks & links)
{
  index_ = config->index;
  links.parent_links.at(config);
}

std::vector<BaseUnit *> BaseUnit::get_child_units() const
{
  std::vector<BaseUnit *> result;
  for (const auto & link : get_child_links()) result.push_back(link->get_child());
  return result;
}

bool BaseUnit::update()
{
  // Update the level of this unit.
  update_status();

  // If the level does not change, it will not affect the parents.
  const auto level = get_level();
  if (level == prev_level_) return false;
  prev_level_ = level;

  // If the level changes, the parents also need to be updated.
  bool result = false;
  for (const auto & link : parents_) {
    const auto unit = link->get_parent();
    result = result || unit->update();
  }
  return result;
}

NodeUnit::NodeUnit(const UnitConfig * config, const GraphLinks & links) : BaseUnit(config, links)
{
  struct_.path = config->path;
  status_.level = DiagnosticStatus::STALE;
}

void NodeUnit::initialize_struct()
{
  struct_.type = get_type();
}

void NodeUnit::initialize_status()
{
  if (get_child_links().size() == 0) update();
}

LeafUnit::LeafUnit(const UnitConfig * config, const GraphLinks & links) : BaseUnit(config, links)
{
  struct_.path = config->path;
  struct_.name = config->data.required("diag").text();
  status_.level = DiagnosticStatus::STALE;
}

void LeafUnit::initialize_struct()
{
  // Do nothing.
}

void LeafUnit::initialize_status()
{
  if (get_child_links().size() == 0) update();
}

DiagUnit::DiagUnit(const UnitConfig * config, const GraphLinks & links) : LeafUnit(config, links)
{
  timeout_ = config->data.optional("timeout").real(1.0);
}

void DiagUnit::update_status()
{
  // Do nothing. The level is updated by on_diag and on_time.
}

bool DiagUnit::on_diag(const rclcpp::Time & stamp, const DiagnosticStatus & status)
{
  last_updated_time_ = stamp;
  status_.level = status.level;
  status_.message = status.message;
  status_.hardware_id = status.hardware_id;
  status_.values = status.values;
  return update();
}

bool DiagUnit::on_time(const rclcpp::Time & stamp)
{
  if (last_updated_time_) {
    const auto updated = last_updated_time_.value();
    const auto elapsed = (stamp - updated).seconds();
    if (timeout_ < elapsed) {
      last_updated_time_ = std::nullopt;
      status_ = DiagLeafStatus();
      status_.level = DiagnosticStatus::STALE;
    }
  }
  return update();
}

MaxUnit::MaxUnit(const UnitConfig * config, const GraphLinks & links) : NodeUnit(config, links)
{
  links_ = children(links, config->list);
}

void MaxUnit::update_status()
{
  DiagnosticLevel level = DiagnosticStatus::OK;
  for (const auto & link : links_) {
    level = std::max(level, link->get_child()->get_level());
  }
  status_.level = std::min(level, DiagnosticStatus::ERROR);
}

void ShortCircuitMaxUnit::update_status()
{
  // TODO(Takagi, Isamu): update link flags.
  DiagnosticLevel level = DiagnosticStatus::OK;
  for (const auto & link : links_) {
    level = std::max(level, link->get_child()->get_level());
  }
  status_.level = std::min(level, DiagnosticStatus::ERROR);
}

MinUnit::MinUnit(const UnitConfig * config, const GraphLinks & links) : NodeUnit(config, links)
{
  links_ = children(links, config->list);
}

void MinUnit::update_status()
{
  DiagnosticLevel level = DiagnosticStatus::OK;
  if (!links_.empty()) {
    level = DiagnosticStatus::STALE;
    for (const auto & link : links_) {
      level = std::min(level, link->get_child()->get_level());
    }
  }
  status_.level = std::min(level, DiagnosticStatus::ERROR);
}

RemapUnit::RemapUnit(const UnitConfig * config, const GraphLinks & links) : NodeUnit(config, links)
{
  link_ = links.config_links.at(config->item);
}

void RemapUnit::update_status()
{
  const auto level = link_->get_child()->get_level();
  status_.level = (level == level_from_) ? level_to_ : level;
}

WarnToOkUnit::WarnToOkUnit(const UnitConfig * config, const GraphLinks & links)
: RemapUnit(config, links)
{
  level_from_ = DiagnosticStatus::WARN;
  level_to_ = DiagnosticStatus::OK;
}

WarnToErrorUnit::WarnToErrorUnit(const UnitConfig * config, const GraphLinks & links)
: RemapUnit(config, links)
{
  level_from_ = DiagnosticStatus::WARN;
  level_to_ = DiagnosticStatus::ERROR;
}

void ConstUnit::update_status()
{
  // Do nothing. This unit always returns the same level.
}

OkUnit::OkUnit(const UnitConfig * config, const GraphLinks & links) : ConstUnit(config, links)
{
  status_.level = DiagnosticStatus::OK;
}

WarnUnit::WarnUnit(const UnitConfig * config, const GraphLinks & links) : ConstUnit(config, links)
{
  status_.level = DiagnosticStatus::WARN;
}

ErrorUnit::ErrorUnit(const UnitConfig * config, const GraphLinks & links) : ConstUnit(config, links)
{
  status_.level = DiagnosticStatus::ERROR;
}

StaleUnit::StaleUnit(const UnitConfig * config, const GraphLinks & links) : ConstUnit(config, links)
{
  status_.level = DiagnosticStatus::STALE;
}

}  // namespace diagnostic_graph_aggregator
