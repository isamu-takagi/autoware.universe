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

#include "core/error.hpp"
#include "core/graph.hpp"
#include "utils.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <tier4_system_msgs/msg/diagnostic_graph.hpp>

#include <gtest/gtest.h>

using namespace system_diagnostic_graph;  // NOLINT(build/namespaces)

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using tier4_system_msgs::msg::DiagnosticGraph;

constexpr auto OK = DiagnosticStatus::OK;
constexpr auto WARN = DiagnosticStatus::WARN;
constexpr auto ERROR = DiagnosticStatus::ERROR;
constexpr auto STALE = DiagnosticStatus::STALE;

struct TestData
{
  std::string config;
  std::vector<uint8_t> levels;
  uint8_t result;
};

DiagnosticArray create_input(const std::vector<uint8_t> & levels)
{
  DiagnosticArray array;
  for (size_t i = 0; i < levels.size(); ++i) {
    DiagnosticStatus status;
    status.level = levels[i];
    status.name = "test: input-" + std::to_string(i);
    array.status.push_back(status);
  }
  return array;
};

bool check_output(const DiagnosticGraph & graph, uint8_t level)
{
  for (const auto & node : graph.nodes) {
    if (node.status.name == "output" && node.status.level == level) {
      return true;
    }
  }
  return false;
}

void check_graph(const TestData & test)
{
  const auto stamp = rclcpp::Clock().now();
  Graph graph;
  graph.init(resource(test.config));
  graph.callback(stamp, create_input(test.levels));
  EXPECT_TRUE(check_output(graph.report(stamp), test.result));
}

TEST(Aggregation, WarnToError1)
{
  TestData test;
  test.config = "test2/warn-to-error.yaml";
  test.levels = {OK};
  test.result = OK;
  check_graph(test);
}

TEST(Aggregation, WarnToError2)
{
  TestData test;
  test.config = "test2/warn-to-error.yaml";
  test.levels = {WARN};
  test.result = ERROR;
  check_graph(test);
}

TEST(Aggregation, WarnToError3)
{
  TestData test;
  test.config = "test2/warn-to-error.yaml";
  test.levels = {ERROR};
  test.result = ERROR;
  check_graph(test);
}

TEST(Aggregation, WarnToError4)
{
  TestData test;
  test.config = "test2/warn-to-error.yaml";
  test.levels = {STALE};
  test.result = STALE;
  check_graph(test);
}
