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

namespace diagnostic_graph_aggregator
{

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

struct UnitConfig
{
  explicit UnitConfig(const TreeData & data) : data(data) {}
  TreeData data;
  std::string type;
  std::string path;
};

struct LinkConfig
{
};

class FileConfig
{
public:
  explicit FileConfig(const PathConfig * path);
  const auto & paths() const { return paths_; }

private:
  void parse_path_config(const TreeData & data);
  void parse_edit_config(const TreeData & data);
  void parse_unit_config(const TreeData & data);
  std::vector<std::unique_ptr<PathConfig>> paths_;
  std::vector<std::unique_ptr<EditConfig>> edits_;
  std::vector<std::unique_ptr<UnitConfig>> units_;
  std::vector<std::unique_ptr<LinkConfig>> links_;
};

class RootConfig
{
public:
  static RootConfig Load(const std::string & path);
  explicit RootConfig(const PathConfig * root);

private:
  std::vector<FileConfig> files_;
};

}  // namespace diagnostic_graph_aggregator

#endif  // COMMON__CONFIG__CONFIG_HPP_
