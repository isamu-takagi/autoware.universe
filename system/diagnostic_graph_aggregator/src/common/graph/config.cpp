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

#include "types/names.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <filesystem>
#include <queue>
#include <regex>
#include <unordered_map>
#include <utility>

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
  link->parent = unit;
  link->child = create_unit_config(data);
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

void apply_links(FileConfig & config)
{
  // Separate units into link types and others.
  std::vector<std::unique_ptr<UnitConfig>> link_units;
  std::vector<std::unique_ptr<UnitConfig>> node_units;
  for (auto & unit : config.units) {
    if (unit->type == unit_name::link) {
      link_units.push_back(std::move(unit));
    } else {
      node_units.push_back(std::move(unit));
    }
  }

  // Create a mapping from path to unit.
  std::unordered_map<std::string, UnitConfig *> path_to_unit;
  for (const auto & unit : node_units) {
    if (unit->path.empty()) {
      continue;
    }
    if (path_to_unit.count(unit->path)) {
      throw PathConflict(unit->path);
    }
    path_to_unit[unit->path] = unit.get();
  }

  // Create a mapping from unit to unit.
  std::unordered_map<UnitConfig *, UnitConfig *> unit_to_unit;
  for (const auto & unit : link_units) {
    const auto path = unit->data.required("link").text();
    const auto pair = path_to_unit.find(path);
    if (pair == path_to_unit.end()) {
      throw PathNotFound(unit->data.path(), path);
    }
    unit_to_unit[unit.get()] = pair->second;
  }

  // Update links.
  for (const auto & link : config.links) {
    link->child = unit_to_unit.at(link->child);
  }

  // TODO(Takagi, Isamu): check graph loop

  // Remove link type units from the graph.
  config.units = std::move(node_units);
}

void apply_edits(FileConfig & config)
{
  (void)config;
  // TODO(Takagi, Isamu)
  /*
  std::unordered_map<std::string, UnitConfig::SharedPtr> paths;
  for (const auto & node : root.nodes) {
    paths[node->path] = node;
  }

  std::unordered_set<UnitConfig::SharedPtr> removes;
  for (const auto & edit : root.edits) {
    if (edit->type == "remove") {
      if (!paths.count(edit->path)) {

        throw PathNotFound(edit->data.path(), path);
      }
      removes.insert(paths.at(edit->path));
    }
  }

  const auto filter = [removes](const UnitConfig::SharedPtrList & nodes) {
    UnitConfig::SharedPtrList result;
    for (const auto & node : nodes) {
      if (!removes.count(node)) {
        result.push_back(node);
      }
    }
    return result;
  };
  for (const auto & node : root.nodes) {
    node->children = filter(node->children);
  }
  root.nodes = filter(root.nodes);
  */
}

FileConfig TreeLoader::flatten()
{
  FileConfig config;
  for (auto & file : files_) file.release(config);
  apply_links(config);
  apply_edits(config);
  return config;
}

}  // namespace diagnostic_graph_aggregator
