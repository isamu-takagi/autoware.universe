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

#include "dump.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

namespace diagnostic_graph_utils
{

DumpNode::DumpNode() : Node("dump")
{
  using std::placeholders::_1;
  sub_graph_.register_create_callback(std::bind(&DumpNode::on_create, this, _1));
  sub_graph_.register_update_callback(std::bind(&DumpNode::on_update, this, _1));
  sub_graph_.subscribe(*this, 1);
}

void DumpNode::on_create(DiagGraph::ConstSharedPtr graph)
{
  // Initialize table line information.
  const std::string title_index = "No";
  const std::string title_level = "Level";
  const std::string title_path = "Path";
  const std::string title_type = "Type";
  const std::string title_link = "Link";

  // Merge nodes and units as units.
  std::vector<DiagUnit *> units;
  for (const auto & node : graph->nodes()) units.push_back(node.get());
  for (const auto & diag : graph->diags()) units.push_back(diag.get());

  // Assign merged units index.
  int table_index = 0;
  for (const auto & unit : units) {
    TableLine line;
    line.index = ++table_index;
    table_.emplace(unit, line);
  }

  // Create link index text. Use text2 as a temporary variable.
  for (const auto & unit : units) {
    std::string links;
    for (const auto & child : unit->children()) {
      links += std::to_string(table_.at(child).index) + " ";
    }
    if (!links.empty()) {
      links.pop_back();
    }
    table_.at(unit).text2 = links;
  }

  // Calculate table cell width.
  const auto width_index = std::to_string(units.size()).length();
  const auto width_level = title_level.length();
  auto width_path = title_path.length();
  auto width_type = title_type.length();
  auto width_link = title_link.length();
  for (const auto & unit : units) {
    const auto & line = table_.at(unit);
    width_path = std::max(width_path, unit->path().length());
    width_type = std::max(width_type, unit->type().length());
    width_link = std::max(width_link, line.text2.length());
  }

  // Create table lines.
  for (const auto & unit : units) {
    auto & line = table_.at(unit);
    std::ostringstream text1;
    std::ostringstream text2;
    text1 << "| " << std::right << std::setw(width_index) << line.index << " |";
    text2 << "| " << std::left << std::setw(width_path) << unit->path() << " ";
    text2 << "| " << std::left << std::setw(width_type) << unit->type() << " ";
    text2 << "| " << std::left << std::setw(width_link) << line.text2 << " |";
    line.text1 = text1.str() + " ";
    line.text2 = " " + text2.str();
  }

  // Create table header.
  {
    std::ostringstream header;
    header << "| " << std::left << std::setw(width_index) << title_index << " ";
    header << "| " << std::left << std::setw(width_level) << title_level << " ";
    header << "| " << std::left << std::setw(width_path) << title_path << " ";
    header << "| " << std::left << std::setw(width_type) << title_type << " ";
    header << "| " << std::left << std::setw(width_link) << title_link << " ";
    header << "|";
    header_ = header.str();

    border_ += "|" + std::string(width_index + 2, '-');
    border_ += "|" + std::string(width_level + 2, '-');
    border_ += "|" + std::string(width_path + 2, '-');
    border_ += "|" + std::string(width_type + 2, '-');
    border_ += "|" + std::string(width_link + 2, '-');
    border_ += "|";
  }
}

void DumpNode::on_update(DiagGraph::ConstSharedPtr graph)
{
  const auto text_level = [](DiagUnit::DiagLevel level) {
    if (level == DiagUnit::DiagStatus::OK) return "OK   ";
    if (level == DiagUnit::DiagStatus::WARN) return "WARN ";
    if (level == DiagUnit::DiagStatus::ERROR) return "ERROR";
    if (level == DiagUnit::DiagStatus::STALE) return "STALE";
    return "-----";
  };

  std::cout << border_ << std::endl;
  std::cout << header_ << std::endl;
  std::cout << border_ << std::endl;

  for (const auto & node : graph->nodes()) {
    const auto & line = table_.at(node.get());
    std::cout << line.text1 << text_level(node->level()) << line.text2 << std::endl;
  }
  for (const auto & diag : graph->diags()) {
    const auto & line = table_.at(diag.get());
    std::cout << line.text1 << text_level(diag->level()) << line.text2 << std::endl;
  }
}

}  // namespace diagnostic_graph_utils

int main(int argc, char ** argv)
{
  using diagnostic_graph_utils::DumpNode;
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<DumpNode>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
