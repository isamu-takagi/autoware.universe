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

#include "expr.hpp"

#include "config.hpp"
#include "graph.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

//
#include <iostream>

namespace system_diagnostic_graph
{

std::unique_ptr<BaseExpr> BaseExpr::create(Graph & graph, YAML::Node yaml)
{
  if (!yaml.IsMap()) {
    throw ConfigError("expr object is not a dict");
  }
  if (!yaml["type"]) {
    throw ConfigError("expr object has no 'type' field");
  }

  const auto type = take<std::string>(yaml, "type");

  if (type == "unit") {
    return std::make_unique<UnitExpr>(graph, yaml);
  }
  if (type == "diag") {
    return std::make_unique<DiagExpr>(graph, yaml);
  }
  if (type == "and") {
    return std::make_unique<AndExpr>(graph, yaml);
  }
  if (type == "or") {
    return std::make_unique<OrExpr>(graph, yaml);
  }
  if (type == "debug-ok") {
    return std::make_unique<ConstExpr>(DiagnosticStatus::OK);
  }
  if (type == "debug-warn") {
    return std::make_unique<ConstExpr>(DiagnosticStatus::WARN);
  }
  if (type == "debug-error") {
    return std::make_unique<ConstExpr>(DiagnosticStatus::ERROR);
  }
  if (type == "debug-stale") {
    return std::make_unique<ConstExpr>(DiagnosticStatus::STALE);
  }
  throw ConfigError("unknown expr type: " + type);
}

DiagnosticLevel ConstExpr::eval() const
{
  return level_;
}

UnitExpr::UnitExpr(Graph & graph, YAML::Node yaml)
{
  if (!yaml["name"]) {
    throw ConfigError("unit object has no 'name' field");
  }
  const auto name = take<std::string>(yaml, "name");
  node_ = graph.find_unit(name);
  if (!node_) {
    throw ConfigError("unit node '" + name + "' does not exist");
  }
}

DiagnosticLevel UnitExpr::eval() const
{
  return DiagnosticStatus::OK;
}

DiagExpr::DiagExpr(Graph & graph, YAML::Node yaml)
{
  if (!yaml["name"]) {
    throw ConfigError("diag object has no 'name' field");
  }
  const auto name = yaml["name"].as<std::string>();
  const auto hardware = yaml["hardware"].as<std::string>();
  node_ = graph.find_diag(name, hardware);
  if (!node_) {
    node_ = graph.make_diag(name, hardware);
  }
}

DiagnosticLevel DiagExpr::eval() const
{
  return DiagnosticStatus::OK;
}

AndExpr::AndExpr(Graph & graph, YAML::Node yaml)
{
  if (!yaml["list"]) {
    throw ConfigError("expr object has no 'list' field");
  }
  if (!yaml["list"].IsSequence()) {
    throw ConfigError("list field is not a list");
  }

  for (const auto & node : yaml["list"]) {
    list_.push_back(BaseExpr::create(graph, node));
  }
}

DiagnosticLevel AndExpr::eval() const
{
  std::vector<DiagnosticLevel> levels;
  for (const auto & expr : list_) {
    levels.push_back(expr->eval());
  }
  const auto level = *std::max_element(levels.begin(), levels.end());
  return std::min(level, DiagnosticStatus::ERROR);
}

OrExpr::OrExpr(Graph & graph, YAML::Node yaml)
{
  if (!yaml["list"]) {
    throw ConfigError("expr object has no 'list' field");
  }
  if (!yaml["list"].IsSequence()) {
    throw ConfigError("list field is not a list");
  }

  for (const auto & node : yaml["list"]) {
    list_.push_back(BaseExpr::create(graph, node));
  }
}

DiagnosticLevel OrExpr::eval() const
{
  std::vector<DiagnosticLevel> levels;
  for (const auto & expr : list_) {
    levels.push_back(expr->eval());
  }
  const auto level = *std::min_element(levels.begin(), levels.end());
  return std::min(level, DiagnosticStatus::ERROR);
}

}  // namespace system_diagnostic_graph
