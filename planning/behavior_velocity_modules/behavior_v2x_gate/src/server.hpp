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

#ifndef SERVER_HPP_
#define SERVER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_v2x_msgs/srv/acquire_gate_lock.hpp>
#include <tier4_v2x_msgs/srv/release_gate_lock.hpp>

#include <lanelet2_core/Forward.h>

#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>

namespace behavior_velocity_planner::v2x_gate
{

struct ClientStatus
{
  std::set<lanelet::Id> gates;
  double distance;
};

struct ServerStatus
{
  std::set<lanelet::Id> gates;
  bool cancel;
};

class LockTarget
{
public:
  void acquire(const ClientStatus & status);
  void release();
  ServerStatus status() const;

private:
  ClientStatus client_status_;
  ServerStatus server_status_;
};

class LockServer
{
public:
  explicit LockServer(rclcpp::Node * node);

private:
  using AcquireGateLock = tier4_v2x_msgs::srv::AcquireGateLock;
  using ReleaseGateLock = tier4_v2x_msgs::srv::ReleaseGateLock;
  rclcpp::Client<AcquireGateLock>::SharedPtr srv_acquire_;
  rclcpp::Client<ReleaseGateLock>::SharedPtr srv_release_;
};

}  // namespace behavior_velocity_planner::v2x_gate

#endif  // SERVER_HPP_
