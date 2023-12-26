// Copyright 2023 The Autoware Contributors
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

#ifndef VEHICLE_CONTROL_INTERFACE__STEERING_STATUS__RAW_HPP_
#define VEHICLE_CONTROL_INTERFACE__STEERING_STATUS__RAW_HPP_

#include <rclcpp/qos.hpp>

#include <autoware_vehicle_msgs/msg/steering_report.hpp>

#include <string>

namespace vehicle_control_interface::steering_status::raw
{

struct Interface
{
  using Message = autoware_vehicle_msgs::msg::SteeringReport;
  using Adaptor = Message;

  static std::string get_name() { return "/interface/name"; }
  static rclcpp::QoS get_pub_qos() { return rclcpp::QoS(1); }
  static rclcpp::QoS get_sub_qos() { return rclcpp::QoS(1); }
};

}  // namespace vehicle_control_interface::steering_status::raw

#endif  // VEHICLE_CONTROL_INTERFACE__STEERING_STATUS__RAW_HPP_
