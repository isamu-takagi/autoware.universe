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
#include <utility>

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
  throw UnknownSubstitution(data.path(), substitution);
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

FileLoader::FileLoader(const PathConfig * path)
{
  if (!std::filesystem::exists(path->resolved)) {
    throw FileNotFound(path->data.path(), path->resolved);
  }
  TreeData tree = TreeData::Load(path->resolved);

  const auto paths = tree.optional("files").children("files");
  const auto edits = tree.optional("edits").children("edits");
  const auto units = tree.optional("nodes").children("nodes");
  for (const auto & data : paths) create_path_config(data);
  for (const auto & data : edits) create_edit_config(data);
  for (const auto & data : units) create_unit_config(data);
}

PathConfig * FileLoader::create_path_config(const TreeData & data)
{
  const auto path = paths_.emplace_back(std::make_unique<PathConfig>(data)).get();
  path->original = path->data.required("path").text();
  path->resolved = resolve_file_path(path->original, data);
  return path;
}

EditConfig * FileLoader::create_edit_config(const TreeData & data)
{
  const auto edit = edits_.emplace_back(std::make_unique<EditConfig>(data)).get();
  edit->type = edit->data.required("type").text();
  return edit;
}

UnitConfig * FileLoader::create_unit_config(const TreeData & data)
{
  const auto unit = units_.emplace_back(std::make_unique<UnitConfig>(data)).get();
  unit->type = unit->data.required("type").text();
  unit->path = unit->data.optional("path").text();

  const auto list = unit->data.optional("list").children();
  for (const auto & data : list) {
    unit->list.push_back(create_link_config(data, unit));
  }
  return unit;
}

LinkConfig * FileLoader::create_link_config(const TreeData & data, UnitConfig * unit)
{
  const auto link = links_.emplace_back(std::make_unique<LinkConfig>()).get();
  link->p = unit;
  link->c = create_unit_config(data);
  return link;
}

void FileLoader::release(FileConfig & config)
{
  for (auto & path : paths_) config.paths.push_back(std::move(path));
  for (auto & edit : edits_) config.edits.push_back(std::move(edit));
  for (auto & unit : units_) config.units.push_back(std::move(unit));
  for (auto & link : links_) config.links.push_back(std::move(link));
}

TreeLoader TreeLoader::Load(const std::string & path)
{
  PathConfig root(TreeData::None());
  root.original = path;
  root.resolved = path;
  return TreeLoader(&root);
}

TreeLoader::TreeLoader(const PathConfig * root)
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

FileConfig TreeLoader::flatten()
{
  FileConfig config;
  for (auto & file : files_) file.release(config);
  return config;
}

}  // namespace diagnostic_graph_aggregator
