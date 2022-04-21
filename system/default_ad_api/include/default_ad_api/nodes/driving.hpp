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

#ifndef DEFAULT_AD_API__NODES__DRIVING_HPP_
#define DEFAULT_AD_API__NODES__DRIVING_HPP_

#include "default_ad_api/specs/driving/engage.hpp"
#include "default_ad_api/specs/driving/state.hpp"
#include "default_ad_api/specs/internal/autoware/get_engage.hpp"
#include "default_ad_api/specs/internal/autoware/get_state.hpp"
#include "default_ad_api/specs/internal/autoware/set_engage.hpp"

#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

namespace default_ad_api
{

class DrivingNode : public rclcpp::Node
{
public:
  explicit DrivingNode(const rclcpp::NodeOptions & options);

private:
  using DrivingEngage = autoware_ad_api_msgs::srv::DrivingEngage;

  // API driving engage
  component_interface_utils::Service<ad_api::driving::engage::T>::SharedPtr srv_api_engage_;
  void onDrivingEngage(
    const DrivingEngage::Request::SharedPtr request,
    const DrivingEngage::Response::SharedPtr response);

  // API driving state
  component_interface_utils::Publisher<ad_api::driving::state::T>::SharedPtr pub_api_state_;

  // set autoware engage
  component_interface_utils::Client<internal_api::autoware::set_engage::T>::SharedPtr
    cli_autoware_engage_;

  // get autoware engage
  component_interface_utils::Subscription<internal_api::autoware::get_engage::T>::SharedPtr
    sub_autoware_engage_;
  void onAutowareEngage(const autoware_auto_vehicle_msgs::msg::Engage::ConstSharedPtr message);

  // get autoware state
  component_interface_utils::Subscription<internal_api::autoware::get_state::T>::SharedPtr
    sub_autoware_state_;
  void onAutowareState(const autoware_auto_system_msgs::msg::AutowareState::ConstSharedPtr message);
};

}  // namespace default_ad_api

#endif  // DEFAULT_AD_API__NODES__DRIVING_HPP_
