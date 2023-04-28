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

#include <vector>

namespace
{

/*
std::string to_string(const std::set<lanelet::Id> & ids)
{
  std::string text;
  for (const auto & id : ids) {
    text += std::to_string(id) + " ";
  }
  return text;
}
*/

std::vector<std::string> to_message_gates(const std::set<lanelet::Id> & ids)
{
  std::vector<std::string> result;
  for (const auto & id : ids) {
    result.push_back(std::to_string(id));
  }
  return result;
}

}  // namespace

namespace behavior_velocity_planner::v2x_gate
{

LockTarget::LockTarget(const std::string & category, const lanelet::Id target)
{
  category_ = category;
  target_ = std::to_string(target);
  cancel_ = false;
  distance_ = 0;
  client_.sequence = 0;
  server_.sequence = 0;
}

ServerStatus LockTarget::status() const
{
  return ServerStatus{};
}

void LockTarget::update(const std::set<lanelet::Id> & gates, double distance)
{
  if (client_.gates != gates) {
    client_.gates = gates;
    client_.sequence += 1;
  }
  distance_ = distance;
}

GateLockClientStatus LockTarget::get_client_status()
{
  GateLockClientStatus status;
  status.target.category = category_;
  status.target.target = target_;
  status.target.gates = to_message_gates(client_.gates);
  status.target.sequence = client_.sequence;
  status.priority = distance_;
  return status;
}

void LockTarget::set_server_status(const GateLockServerStatus & status)
{
  (void)status;
}

/*
void LockTarget::update(LockServer & server)
{
  const auto logger = rclcpp::get_logger("behavior_velocity_planner.v2x_gate.status");
  RCLCPP_INFO_STREAM(logger, "status update");
  RCLCPP_INFO_STREAM(logger, " - client gates: " << to_string(client_status_.gates));
  RCLCPP_INFO_STREAM(logger, " - server gates: " << to_string(server_status_.gates));

  if (client_status_.gates != server_status_.gates) {
    if (client_status_.gates.empty()) {
      RCLCPP_INFO_STREAM(logger, " - request release");
    } else {
      if (!acquire_request_) {
        RCLCPP_INFO_STREAM(logger, " - request acquire");
        const auto srv = server.srv_acquire();
        if (srv->service_is_ready()) {
          const auto req = std::make_shared<AcquireGateLock::Request>();
          req->target.category = category_;
          req->target.target = target_;
          req->target.gates = to_message_gates(client_status_.gates);
          req->priority = client_status_.distance;
          srv->async_send_request(
            req, std::bind(&LockTarget::on_acquire_response, this, std::placeholders::_1));
          acquire_request_ = req;
          RCLCPP_INFO_STREAM(logger, " - request acquire done");
        }
      }
    }
  }
}
*/

}  // namespace behavior_velocity_planner::v2x_gate
