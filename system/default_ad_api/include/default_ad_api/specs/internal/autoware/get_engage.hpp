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

#ifndef DEFAULT_AD_API__SPECS__INTERNAL__AUTOWARE__GET_ENGAGE_HPP_
#define DEFAULT_AD_API__SPECS__INTERNAL__AUTOWARE__GET_ENGAGE_HPP_

#include <rclcpp/qos.hpp>

#include <autoware_auto_vehicle_msgs/msg/engage.hpp>

namespace internal_api::autoware::get_engage
{

struct T
{
  using Message = autoware_auto_vehicle_msgs::msg::Engage;
  static constexpr char name[] = "/api/autoware/get/engage";
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
};

}  // namespace internal_api::autoware::get_engage

#endif  // DEFAULT_AD_API__SPECS__INTERNAL__AUTOWARE__GET_ENGAGE_HPP_
