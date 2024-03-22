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

#ifndef COMMAND__DIAGNOSTIC_HPP_
#define COMMAND__DIAGNOSTIC_HPP_

#include "interface.hpp"

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>

namespace vehicle_cmd_gate2
{

class DiagnosticBridge : public CommandBridge
{
public:
  using DiagnosticStatusWrapper = diagnostic_updater::DiagnosticStatusWrapper;
  DiagnosticBridge();
  void on_diag_task(DiagnosticStatusWrapper & stat);
};

}  // namespace vehicle_cmd_gate2

#endif  // COMMAND__DIAGNOSTIC_HPP_
