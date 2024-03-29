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

#include "diagnostic_graph_utils/graph.hpp"

namespace diagnostic_graph_utils
{

void DiagGraph::create(const DiagGraphStruct & msg)
{
  name_ = msg.name;
  for (const auto & node : msg.nodes) nodes_.push_back(std::make_unique<DiagNode>(node));
  for (const auto & diag : msg.diags) diags_.push_back(std::make_unique<DiagLeaf>(diag));
}

bool DiagGraph::update(const DiagGraphStatus & msg)
{
  if (name_ != msg.name) return false;
  for (size_t i = 0; i < msg.nodes.size(); ++i) nodes_[i]->update(msg.nodes[i]);
  for (size_t i = 0; i < msg.diags.size(); ++i) diags_[i]->update(msg.diags[i]);
  return true;
}

}  // namespace diagnostic_graph_utils
