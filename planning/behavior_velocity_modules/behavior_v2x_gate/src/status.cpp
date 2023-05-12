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

std::vector<std::string> to_message_lines(const std::set<lanelet::Id> & ids)
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

LockTarget::LockTarget(const std::string & category, const lanelet::Id area)
{
  category_ = category;
  area_ = std::to_string(area);
  cancel_ = false;
  distance_ = 0;

  SyncStatus status;
  status.lines = std::set<lanelet::Id>();
  status.sequence = 0;
  status_.push_back(status);
}

ServerStatus LockTarget::status() const
{
  const auto & server = status_.front();
  ServerStatus status;
  status.lines = server.lines;
  status.cancel = cancel_;
  return status;
}

void LockTarget::update(const std::set<lanelet::Id> & lines, double distance)
{
  const auto & latest = status_.back();
  if (latest.lines != lines) {
    SyncStatus status;
    status.lines = lines;
    status.sequence = latest.sequence + 1;
    status_.push_back(status);
  }
  distance_ = distance;
}

GateLockClientStatus LockTarget::get_client_status()
{
  const auto & latest = status_.back();
  GateLockClientStatus status;
  status.target.category = category_;
  status.target.area = area_;
  status.target.lines = to_message_lines(latest.lines);
  status.target.sequence = latest.sequence;
  status.priority = distance_;
  return status;
}

void LockTarget::set_server_status(const GateLockServerStatus & status)
{
  const auto iter = std::find_if(status_.begin(), status_.end(), [status](auto s) {
    return s.sequence == status.target.sequence;
  });
  if (iter != status_.end()) {
    status_.erase(status_.begin(), iter);
    cancel_ = status.cancel;
  }
}

}  // namespace behavior_velocity_planner::v2x_gate
