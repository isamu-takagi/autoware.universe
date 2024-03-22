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

#include "diagnostic.hpp"

namespace vehicle_cmd_gate2
{

DiagnosticBridge::DiagnosticBridge()
{
}

void DiagnosticBridge::on_diag_task(DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  stat.summary(DiagnosticStatus::OK, "test message");
  stat.add<double>("freq control", 12.3);
}

}  // namespace vehicle_cmd_gate2
