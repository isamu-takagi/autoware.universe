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

#ifndef STATUS_HPP_
#define STATUS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_v2x_msgs/msg/gate_lock_client_status_array.hpp>
#include <tier4_v2x_msgs/msg/gate_lock_server_status_array.hpp>

#include <lanelet2_core/Forward.h>

#include <deque>
#include <memory>
#include <set>
#include <string>
#include <utility>

namespace behavior_velocity_planner::v2x_gate
{

using tier4_v2x_msgs::msg::GateLockClientStatus;
using tier4_v2x_msgs::msg::GateLockClientStatusArray;
using tier4_v2x_msgs::msg::GateLockServerStatus;
using tier4_v2x_msgs::msg::GateLockServerStatusArray;

struct ServerStatus
{
  std::set<lanelet::Id> gates;
  bool cancel;
};

struct SyncStatus
{
  std::set<lanelet::Id> gates;
  uint64_t sequence;
};

class LockTarget
{
public:
  LockTarget(const std::string & category, const lanelet::Id target);
  ServerStatus status() const;
  void update(const std::set<lanelet::Id> & gates, double distance);

  GateLockClientStatus get_client_status();
  void set_server_status(const GateLockServerStatus & status);
  auto get_key() { return std::make_pair(category_, target_); }

private:
  std::string category_;
  std::string target_;
  std::deque<SyncStatus> status_;
  bool cancel_;
  double distance_;
};

}  // namespace behavior_velocity_planner::v2x_gate

#endif  // STATUS_HPP_
