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

#ifndef MOTION_HPP_
#define MOTION_HPP_

#include "default_ad_api/specs/driving/state.hpp"
#include "default_ad_api/specs/internal/engage/get.hpp"
#include "default_ad_api/specs/internal/engage/set.hpp"
#include "default_ad_api/specs/motion/state.hpp"
#include "utils/types.hpp"

#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

namespace default_ad_api
{

class MotionNode : public rclcpp::Node
{
public:
  explicit MotionNode(const rclcpp::NodeOptions & options);

private:
  // AD API
  Publisher<ad_api::motion::state::T>::SharedPtr pub_motion_state_;

  // interfaces
  Client<internal_api::engage::set::T>::SharedPtr cli_autoware_engage_;
  Subscription<internal_api::engage::get::T>::SharedPtr sub_autoware_engage_;
  Subscription<ad_api::driving::state::T>::SharedPtr sub_driving_state_;
};

}  // namespace default_ad_api

#endif  // MOTION_HPP_
