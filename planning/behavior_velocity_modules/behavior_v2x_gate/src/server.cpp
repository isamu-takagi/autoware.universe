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

#include "server.hpp"

namespace behavior_velocity_planner::v2x_gate
{

void LockTarget::acquire(const ClientStatus & status)
{
  client_status_ = status;
}

void LockTarget::release()
{
}

ServerStatus LockTarget::status() const
{
  return ServerStatus{};
}

LockServer::LockServer(rclcpp::Node * node)
{
  srv_acquire_ = node->create_client<AcquireGateLock>("/test/gate_lock/acquire");
  srv_release_ = node->create_client<ReleaseGateLock>("/test/gate_lock/release");
}

}  // namespace behavior_velocity_planner::v2x_gate
