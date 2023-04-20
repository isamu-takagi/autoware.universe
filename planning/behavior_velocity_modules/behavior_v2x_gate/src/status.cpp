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

#include "status.hpp"

namespace
{

std::string to_string(const std::set<lanelet::Id> & ids)
{
  std::string text;
  for (const auto & id : ids) {
    text += std::to_string(id) + " ";
  }
  return text;
}

}  // namespace

namespace behavior_velocity_planner::v2x_gate
{

void LockTarget::acquire(const ClientStatus & status)
{
  client_status_ = status;
}

void LockTarget::release()
{
}

void LockTarget::update(LockServer & server)
{
  const auto logger = rclcpp::get_logger("behavior_velocity_planner.v2x_gate.status");
  RCLCPP_INFO_STREAM(logger, "status update");
  RCLCPP_INFO_STREAM(logger, " - client gates: " << to_string(client_status_.gates));
  RCLCPP_INFO_STREAM(logger, " - server gates: " << to_string(server_status_.gates));

  if (client_status_.gates != server_status_.gates) {
    RequestStatus status;
    status.gates = client_status_.gates;
    status.stamp = server.now();

    if (client_status_.gates.empty()) {
      RCLCPP_INFO_STREAM(logger, " - request release");
    } else {
      RCLCPP_INFO_STREAM(logger, " - request acquire");
    }
  }

  (void)server;
}

ServerStatus LockTarget::status() const
{
  return ServerStatus{};
}

LockServer::LockServer(rclcpp::Node * node)
{
  clock_ = node->get_clock();
  srv_acquire_ = node->create_client<AcquireGateLock>("/test/gate_lock/acquire");
  srv_release_ = node->create_client<ReleaseGateLock>("/test/gate_lock/release");
}

}  // namespace behavior_velocity_planner::v2x_gate
