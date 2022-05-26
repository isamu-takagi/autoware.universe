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

#ifndef DRIVING_HPP_
#define DRIVING_HPP_

#include "default_ad_api/specs/driving/engage.hpp"
#include "default_ad_api/specs/driving/state.hpp"
#include "default_ad_api/specs/internal/autoware/state.hpp"
#include "utils/types.hpp"

#include <rclcpp/rclcpp.hpp>

namespace default_ad_api
{

class DrivingNode : public rclcpp::Node
{
public:
  explicit DrivingNode(const rclcpp::NodeOptions & options);

private:
  // AD API
  Service<ad_api::driving::engage::T>::SharedPtr srv_driving_engage_;
  Publisher<ad_api::driving::state::T>::SharedPtr pub_driving_state_;

  // interfaces
  Subscription<internal_api::autoware::state::T>::SharedPtr sub_autoware_state_;

  // states
  autoware_ad_api_msgs::msg::DrivingState driving_state_;
};

}  // namespace default_ad_api

#endif  // DRIVING_HPP_
