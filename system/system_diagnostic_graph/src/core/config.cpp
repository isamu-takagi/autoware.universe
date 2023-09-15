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

#include "config.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace system_diagnostic_graph
{

template <class T>
T take(YAML::Node yaml, const std::string & field)
{
  const auto result = yaml[field].as<T>();
  yaml.remove(field);
  return result;
}

YAML::Node take(ConfigDict & dict, const std::string & field)
{
  const auto result = dict.at(field);
  dict.erase(field);
  return result;
}

ConfigError create_error(const FileConfig & config, const std::string & message)
{
  const std::string marker = config ? "File:" + config->path : "Parameter";
  return ConfigError(message + " (" + marker + ")");
}

ConfigError create_error(const NodeConfig & config, const std::string & message)
{
  const std::string marker = "File:" + config->file + ", Node:" + config->path;
  return ConfigError(message + " (" + marker + ")");
}

NodeConfig parse_config_node(YAML::Node yaml, const FileConfig & scope)
{
  if (!yaml.IsMap()) {
    throw create_error(scope, "node object is not a dict");
  }
  if (!yaml["path"]) {
    throw create_error(scope, "node object has no 'name' field");
  }

  const auto config = std::make_shared<NodeConfig_>();
  config->file = scope->path;
  config->path = take<std::string>(yaml, "path");
  config->yaml = yaml;
  return config;
}

FileConfig parse_config_path(YAML::Node yaml, const FileConfig & scope)
{
  if (!yaml.IsMap()) {
    throw create_error(scope, "file object is not a dict");
  }
  if (!yaml["package"]) {
    throw create_error(scope, "file object has no 'package' field");
  }
  if (!yaml["path"]) {
    throw create_error(scope, "file object has no 'path' field");
  }

  const auto package_name = yaml["package"].as<std::string>();
  const auto package_path = ament_index_cpp::get_package_share_directory(package_name);
  return parse_config_path(package_path + "/" + yaml["path"].as<std::string>(), scope);
}

FileConfig parse_config_path(const std::string & path, const FileConfig & scope)
{
  if (!std::filesystem::exists(path)) {
    throw create_error(scope, "config file '" + path + "' does not exist");
  }
  return parse_config_file(path);
}

FileConfig parse_config_file(const std::string & path)
{
  const auto config = std::make_shared<FileConfig_>();
  config->path = path;

  const auto yaml = YAML::LoadFile(path);
  if (!yaml.IsMap()) {
    throw create_error(config, "config file is not a dict");
  }

  const auto files = yaml["files"] ? yaml["files"] : YAML::Node(YAML::NodeType::Sequence);
  if (!files.IsSequence()) {
    throw create_error(config, "files section is not a list");
  }
  for (const auto file : files) {
    config->files.push_back(parse_config_path(file, config));
  }

  const auto nodes = yaml["nodes"] ? yaml["nodes"] : YAML::Node(YAML::NodeType::Sequence);
  if (!nodes.IsSequence()) {
    throw create_error(config, "nodes section is not a list");
  }
  for (const auto node : nodes) {
    config->nodes.push_back(parse_config_node(node, config));
  }

  return config;
}

void walk_config_tree(const FileConfig & config, std::vector<NodeConfig> & nodes)
{
  nodes.insert(nodes.end(), config->nodes.begin(), config->nodes.end());
  for (const auto & file : config->files) walk_config_tree(file, nodes);
}

std::vector<NodeConfig> load_config_file(const std::string & path)
{
  std::vector<NodeConfig> nodes;
  walk_config_tree(parse_config_path(path, nullptr), nodes);
  return nodes;
}

ExprConfig parse_expr_object(YAML::Node yaml)
{
  if (!yaml.IsMap()) {
    throw ConfigError("expr object is not a dict");
  }
  if (!yaml["type"]) {
    throw ConfigError("expr object has no 'type' field");
  }

  ExprConfig config;
  config.type = take<std::string>(yaml, "type");
  for (const auto & kv : yaml) {
    config.dict[kv.first.as<std::string>()] = kv.second;
  }
  return config;
}

std::string take_expr_text(ConfigDict & dict, const std::string & name)
{
  if (!dict.count(name)) {
    throw ConfigError("expr object has no '" + name + "' field");
  }
  return take(dict, name).as<std::string>();
}

std::string take_expr_text(ConfigDict & dict, const std::string & name, const std::string & fail)
{
  if (!dict.count(name)) {
    return fail;
  }
  return take(dict, name).as<std::string>();
}

std::vector<YAML::Node> take_expr_list(ConfigDict & dict, const std::string & name)
{
  if (!dict["list"]) {
    throw ConfigError("expr object has no '" + name + "' field");
  }

  const auto list = take(dict, name);
  if (!list.IsSequence()) {
    throw ConfigError("expr object " + name + " field is not a list");
  }

  std::vector<YAML::Node> result;
  for (const auto & node : list) {
    result.push_back(node);
  }
  return result;
}

}  // namespace system_diagnostic_graph
