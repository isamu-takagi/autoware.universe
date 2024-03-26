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

#ifndef COMMON__GRAPH__TYPES__LOADER_HPP_
#define COMMON__GRAPH__TYPES__LOADER_HPP_

#include "units.hpp"

#include <vector>

namespace diagnostic_graph_aggregator
{

class Linker
{
public:
  virtual std::vector<UnitLink *> get_parent_links() = 0;
  virtual std::vector<UnitLink *> get_child_links() = 0;
};

}  // namespace diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__TYPES__LOADER_HPP_
