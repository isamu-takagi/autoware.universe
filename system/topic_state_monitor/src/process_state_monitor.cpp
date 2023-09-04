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

#include "process_state_monitor.hpp"

#include <string>
#include <vector>

namespace topic_state_monitor
{
ProcessStateMonitor::ProcessStateMonitor(const rclcpp::NodeOptions & options)
: Node("process_state_monitor", options), updater_(this, 1.0)
{
  const auto nodes = declare_parameter<std::vector<std::string>>("nodes");
  for (const auto & node : nodes) {
    latest_stamps_[node] = std::nullopt;
  }
  timeout_ = declare_parameter<double>("timeout");

  updater_.setHardwareID("process_state_monitor");
  updater_.add("nodes", this, &ProcessStateMonitor::on_diag);
}

void ProcessStateMonitor::on_diag(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  const auto nodes = get_node_names();
  const auto stamp = now();
  for (const auto & node : nodes) {
    auto iter = latest_stamps_.find(node);
    if (iter != latest_stamps_.end()) {
      iter->second = stamp;
    }
  }

  bool is_ok = true;
  for (const auto & [node, latest] : latest_stamps_) {
    if (!latest) {
      is_ok = false;
      stat.addf(node, "Not exist");
      continue;
    }
    const auto seconds = (stamp - latest.value()).seconds();
    if (timeout_ < seconds) {
      is_ok = false;
      stat.addf(node, "Timeout %.2f", seconds);
      continue;
    }
    stat.addf(node, "OK");
  }

  if (is_ok) {
    stat.summary(DiagnosticStatus::OK, "OK");
  } else {
    stat.summary(DiagnosticStatus::ERROR, "Error");
  }
}

}  // namespace topic_state_monitor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(topic_state_monitor::ProcessStateMonitor)
