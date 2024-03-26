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

#ifndef COMMON__CONFIG__CONFIG_HPP_
#define COMMON__CONFIG__CONFIG_HPP_

#include "data.hpp"

#include <memory>
#include <string>
#include <vector>

// DEBUG
#include <iostream>

namespace diagnostic_graph_aggregator
{

struct PathConfig;
struct EditConfig;
struct UnitConfig;
struct LinkConfig;

struct PathConfig
{
  explicit PathConfig(const TreeData & data) : data(data) {}
  TreeData data;
  std::string original;
  std::string resolved;
};

struct EditConfig
{
  explicit EditConfig(const TreeData & data) : data(data) {}
  TreeData data;
  std::string type;
};

struct LinkConfig
{
  using Item = LinkConfig *;
  using List = std::vector<LinkConfig::Item>;
  UnitConfig * parent;
  UnitConfig * child;
};

struct UnitConfig
{
  explicit UnitConfig(const TreeData & data) : data(data) {}
  TreeData data;
  std::string type;
  std::string path;
  LinkConfig::Item item;
  LinkConfig::List list;

  ~UnitConfig() { std::cout << "remove unit: " << type << std::endl; }
};

struct FileConfig
{
  std::vector<std::unique_ptr<PathConfig>> paths;
  std::vector<std::unique_ptr<EditConfig>> edits;
  std::vector<std::unique_ptr<UnitConfig>> units;
  std::vector<std::unique_ptr<LinkConfig>> links;
};

class FileLoader
{
public:
  explicit FileLoader(const PathConfig * path);
  const auto & paths() const { return paths_; }
  void release(FileConfig & config);

private:
  PathConfig * create_path_config(const TreeData & data);
  EditConfig * create_edit_config(const TreeData & data);
  UnitConfig * create_unit_config(const TreeData & data);
  LinkConfig * create_link_config(const TreeData & data, UnitConfig * unit);
  std::vector<std::unique_ptr<PathConfig>> paths_;
  std::vector<std::unique_ptr<EditConfig>> edits_;
  std::vector<std::unique_ptr<UnitConfig>> units_;
  std::vector<std::unique_ptr<LinkConfig>> links_;
};

class TreeLoader
{
public:
  static TreeLoader Load(const std::string & path);
  explicit TreeLoader(const PathConfig * root);
  FileConfig flatten();

private:
  std::vector<FileLoader> files_;
};

}  // namespace diagnostic_graph_aggregator

#endif  // COMMON__CONFIG__CONFIG_HPP_
