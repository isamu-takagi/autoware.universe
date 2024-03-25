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
#include <queue>
#include <regex>

// DEBUG
#include <iostream>

namespace diagnostic_graph_aggregator
{

std::string resolve_substitution(const std::string & substitution, const TreeData & data)
{
  std::stringstream ss(substitution);
  std::string word;
  std::vector<std::string> words;
  while (getline(ss, word, ' ')) {
    words.push_back(word);
  }

  if (words.size() == 2 && words[0] == "find-pkg-share") {
    return ament_index_cpp::get_package_share_directory(words[1]);
  }
  if (words.size() == 1 && words[0] == "dirname") {
    return std::filesystem::path(data.path().file()).parent_path();
  }
  throw UnknownSubstitution(substitution, data.path());
}

std::string resolve_file_path(const std::string & path, const TreeData & data)
{
  static const std::regex pattern(R"(\$\(([^()]*)\))");
  std::smatch m;
  std::string result = path;
  while (std::regex_search(result, m, pattern)) {
    const std::string prefix = m.prefix();
    const std::string suffix = m.suffix();
    result = prefix + resolve_substitution(m.str(1), data) + suffix;
  }
  return result;
}

FileConfig::FileConfig(const PathConfig * path)
{
  std::cout << "==================== load_file ====================" << std::endl;
  std::cout << path->original << std::endl;
  std::cout << path->resolved << std::endl;

  TreeData tree = TreeData::Load(path->resolved);
  const auto paths = tree.optional("files").children("files");
  const auto edits = tree.optional("edits").children("edits");
  const auto units = tree.optional("nodes").children("nodes");
  for (const auto & data : paths) parse_path_config(data);
  for (const auto & data : edits) parse_edit_config(data);
  for (const auto & data : units) parse_unit_config(data);

  std::cout << "----- files -----" << std::endl;
  for (const auto & data : paths_) data->data.dump();

  std::cout << "----- edits -----" << std::endl;
  for (const auto & data : edits_) data->data.dump();

  std::cout << "----- units -----" << std::endl;
  for (const auto & data : units_) data->data.dump();
}

void FileConfig::parse_path_config(const TreeData & data)
{
  const auto path = paths_.emplace_back(std::make_unique<PathConfig>(data)).get();
  path->original = path->data.required("path").text();
  path->resolved = resolve_file_path(path->original, data);
}

void FileConfig::parse_edit_config(const TreeData & data)
{
  const auto edit = edits_.emplace_back(std::make_unique<EditConfig>(data)).get();
  edit->type = edit->data.required("type").text();
}

void FileConfig::parse_unit_config(const TreeData & data)
{
  const auto unit = units_.emplace_back(std::make_unique<UnitConfig>(data)).get();
  unit->type = unit->data.required("type").text();
  unit->path = unit->data.optional("path").text();

  const auto list = unit->data.optional("list").children();
  for (const auto & data : list) {
    parse_unit_config(data);
  }
}

RootConfig::RootConfig(const PathConfig * root)
{
  std::queue<const PathConfig *> paths;
  paths.push(root);

  // TODO(Takagi, Isamu): check include loop.
  while (!paths.empty()) {
    files_.emplace_back(paths.front());
    paths.pop();
    for (const auto & path : files_.back().paths()) {
      paths.push(path.get());
    }
  }
}

RootConfig RootConfig::Load(const std::string & path)
{
  PathConfig root(TreeData::None());
  root.original = path;
  root.resolved = path;
  return RootConfig(&root);
}

}  // namespace diagnostic_graph_aggregator
