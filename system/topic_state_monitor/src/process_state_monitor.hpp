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

#ifndef PROCESS_STATE_MONITOR_HPP_
#define PROCESS_STATE_MONITOR_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <unordered_map>

namespace topic_state_monitor
{

class ProcessStateMonitor : public rclcpp::Node
{
public:
  explicit ProcessStateMonitor(const rclcpp::NodeOptions & options);

private:
  diagnostic_updater::Updater updater_;
  void on_diag(diagnostic_updater::DiagnosticStatusWrapper & stat);

  double timeout_;
  std::unordered_map<std::string, std::optional<rclcpp::Time>> latest_stamps_;
};
}  // namespace topic_state_monitor

#endif  // PROCESS_STATE_MONITOR_HPP_
