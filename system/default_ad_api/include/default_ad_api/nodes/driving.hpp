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
  using DrivingState = autoware_ad_api_msgs::msg::DrivingState;

  component_interface_utils::Service<ad_api::driving::engage::T>::SharedPtr srv_;
  void onDrivingEngage(
    const DrivingEngage::Request::SharedPtr request,
    const DrivingEngage::Response::SharedPtr response);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<DrivingState>::SharedPtr pub_;
  void onTimer();
};

}  // namespace default_ad_api

#endif  // DEFAULT_AD_API__NODES__DRIVING_HPP_
