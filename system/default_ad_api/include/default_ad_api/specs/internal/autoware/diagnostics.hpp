// Copyright 2022 TIER IV, Inc.
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

#ifndef DEFAULT_AD_API__SPECS__INTERNAL__AUTOWARE__DIAGNOSTICS_HPP_
#define DEFAULT_AD_API__SPECS__INTERNAL__AUTOWARE__DIAGNOSTICS_HPP_

#include <rclcpp/qos.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

namespace internal_api::diagnostics
{

struct T
{
  using Message = diagnostic_msgs::msg::DiagnosticArray;
  static constexpr char name[] = "/diagnostics";
  static constexpr size_t depth = 100;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

}  // namespace internal_api::diagnostics

#endif  // DEFAULT_AD_API__SPECS__INTERNAL__AUTOWARE__DIAGNOSTICS_HPP_
