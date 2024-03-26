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

UnitLink::UnitLink(BaseUnit * parent, BaseUnit * child)
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
  status_.used = true;
}

std::vector<BaseUnit *> BaseUnit::get_child_units() const
{
  std::vector<BaseUnit *> result;
  for (const auto & link : get_child_links()) result.push_back(link->get_child());
  return result;
}

void BaseUnit::initialize_children(Linker &)
{
  // Do nothing by default.
}

void BaseUnit::initialize_parents(Linker & linker)
{
  parents_ = linker.take_parents(this);
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

NodeUnit::NodeUnit(const UnitConfigItem & config)
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

LeafUnit::LeafUnit(const UnitConfigItem & config)
{
  struct_.path = config->path;
  struct_.name = config->data.required("diag").text();
  status_.level = DiagnosticStatus::STALE;
}

void LeafUnit::initialize_struct()
{
  // Do nothing by default.
}

void LeafUnit::initialize_status()
{
  if (get_child_links().size() == 0) update();
}

DiagUnit::DiagUnit(const UnitConfigItem & config) : LeafUnit(config)
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

MaxUnit::MaxUnit(const UnitConfigItem & config) : NodeUnit(config)
{
}

void MaxUnit::initialize_children(Linker & linker)
{
  links_ = linker.take_child_list(this);
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

MinUnit::MinUnit(const UnitConfigItem & config) : NodeUnit(config)
{
}

void MinUnit::initialize_children(Linker & linker)
{
  links_ = linker.take_child_list(this);
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

ConstUnit::ConstUnit(const UnitConfigItem & config, DiagnosticLevel level) : NodeUnit(config)
{
  status_.level = level;
}

void ConstUnit::update_status()
{
  // Do nothing. This unit always returns the same level.
}

OkUnit::OkUnit(const UnitConfigItem & config) : ConstUnit(config, DiagnosticStatus::OK)
{
}

WarnUnit::WarnUnit(const UnitConfigItem & config) : ConstUnit(config, DiagnosticStatus::WARN)
{
}

ErrorUnit::ErrorUnit(const UnitConfigItem & config) : ConstUnit(config, DiagnosticStatus::ERROR)
{
}

StaleUnit::StaleUnit(const UnitConfigItem & config) : ConstUnit(config, DiagnosticStatus::STALE)
{
}

}  // namespace diagnostic_graph_aggregator
