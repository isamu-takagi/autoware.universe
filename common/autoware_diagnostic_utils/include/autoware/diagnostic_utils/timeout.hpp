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

#ifndef AUTOWARE__DIAGNOSTIC_UTILS__TIMEOUT_HPP_
#define AUTOWARE__DIAGNOSTIC_UTILS__TIMEOUT_HPP_

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <string>

namespace autoware::diagnostic_utils
{

class TimeoutDiag : public diagnostic_updater::DiagnosticTask
{
public:
  struct Params
  {
    double warn_duration_;
    double error_duration_;
  };

public:
  TimeoutDiag(const Params & params, const rclcpp::Clock & clock, const std::string & name);
  void update(rclcpp::Time stamp);

private:
  void run(diagnostic_updater::DiagnosticStatusWrapper & stat) override;

  const Params params_;
  rclcpp::Clock clock_;
  std::optional<rclcpp::Time> last_stamp_;
};

}  // namespace autoware::diagnostic_utils

#endif  // AUTOWARE__DIAGNOSTIC_UTILS__TIMEOUT_HPP_
