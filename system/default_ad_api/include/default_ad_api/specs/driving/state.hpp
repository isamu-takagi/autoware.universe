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

#ifndef DEFAULT_AD_API__SPECS__DRIVING__STATE_HPP_
#define DEFAULT_AD_API__SPECS__DRIVING__STATE_HPP_

#include <rclcpp/qos.hpp>

#include <autoware_ad_api_msgs/msg/driving_state.hpp>

namespace ad_api::driving::state
{

struct T
{
  using Message = autoware_ad_api_msgs::msg::DrivingState;
  static constexpr char name[] = "/api/driving/state";
  static constexpr double default_hz = 0.0;
  static constexpr size_t depth = 1;
  static constexpr auto reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  static constexpr auto durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
};

}  // namespace ad_api::driving::state

#endif  // DEFAULT_AD_API__SPECS__DRIVING__STATE_HPP_
