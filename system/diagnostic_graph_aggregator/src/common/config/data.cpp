// Copyright 2024 The Autoware Contributors
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

#include "data.hpp"

// DEBUG
#include <iostream>

namespace diagnostic_graph_aggregator
{

TreeData TreeData::Load(const std::string & path)
{
  return TreeData(YAML::LoadFile(path), TreePath(path));
}

TreeData TreeData::None()
{
  return TreeData(YAML::Node(), TreePath(""));
}

TreeData::TreeData(const YAML::Node & yaml, const TreePath & path) : path_(path)
{
  yaml_ = yaml;
}

TreeData::Item TreeData::required(const std::string & name)
{
  // TODO(Takagi, Isamu): check map type.
  const auto path = path_.field(name);
  if (!yaml_[name]) {
    throw FieldNotFound(path);
  }
  const auto data = yaml_[name];
  yaml_.remove(name);
  return TreeData(data, path);
}

TreeData::Item TreeData::optional(const std::string & name)
{
  // TODO(Takagi, Isamu): check map type.
  const auto path = path_.field(name);
  if (!yaml_[name]) {
    return TreeData({}, path);
  }
  const auto data = yaml_[name];
  yaml_.remove(name);
  return TreeData(data, path);
}

TreeData::List TreeData::children(const std::string & path)
{
  if (yaml_.Type() == YAML::NodeType::Null) {
    return {};
  }
  if (yaml_.Type() != YAML::NodeType::Sequence) {
    throw InvalidType(path_);
  }

  TreeData::List result;
  for (size_t i = 0; i < yaml_.size(); ++i) {
    result.emplace_back(yaml_[i], path_.child(path, i));
  }
  return result;
}

std::string TreeData::text(const std::string & fail)
{
  return yaml_.as<std::string>(fail);
}

void TreeData::dump() const
{
  std::cout << "file: " << path_.file() << std::endl;
  std::cout << "path: " << path_.node() << std::endl;
}

}  // namespace diagnostic_graph_aggregator
