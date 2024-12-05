// Copyright 2024 The Autoware Contributors
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

#include "diagnostics.hpp"

namespace autoware::control_cmd_gate
{

CommandDiagnostics::CommandDiagnostics(
  CommandOutput * output, diagnostic_updater::Updater & updater, const TimeoutDiag::Params & params,
  const rclcpp::Clock & clock, const std::string & name)
: CommandBridge(output), timeout_(params, clock, "timeout_" + name)
{
  updater.add(timeout_);
}

void CommandDiagnostics::on_control(const Control::ConstSharedPtr msg)
{
  CommandBridge::on_control(msg);
}

}  // namespace autoware::control_cmd_gate
