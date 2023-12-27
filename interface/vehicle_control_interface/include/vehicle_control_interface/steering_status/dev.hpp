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

#ifndef VEHICLE_CONTROL_INTERFACE__STEERING_STATUS__DEV_HPP_
#define VEHICLE_CONTROL_INTERFACE__STEERING_STATUS__DEV_HPP_

#include <rclcpp/qos.hpp>
#include <rclcpp/type_adapter.hpp>

#include <autoware_vehicle_msgs/msg/steering_report.hpp>

#include <string>

namespace vehicle_control_interface::steering_status::dev
{

struct CustomMessage
{
  builtin_interfaces::msg::Time stamp;
  float steering_tire_angle;
};

struct Interface
{
  using Message = CustomMessage;
  using Adaptor = rclcpp::TypeAdapter<CustomMessage, autoware_vehicle_msgs::msg::SteeringReport>;

  static std::string get_name() { return "/interface/name"; }
  static rclcpp::QoS get_pub_qos() { return rclcpp::QoS(1); }
  static rclcpp::QoS get_sub_qos() { return rclcpp::QoS(1); }
};

}  // namespace vehicle_control_interface::steering_status::dev

template <>
struct rclcpp::TypeAdapter<
  vehicle_control_interface::steering_status::dev::CustomMessage,
  autoware_vehicle_msgs::msg::SteeringReport>
{
  using is_specialized = std::true_type;
  using custom_type = vehicle_control_interface::steering_status::dev::CustomMessage;
  using ros_message_type = autoware_vehicle_msgs::msg::SteeringReport;

  static void convert_to_ros_message(const custom_type & custom, ros_message_type & rosidl)
  {
    rosidl.stamp = custom.stamp;
    rosidl.steering_tire_angle = custom.steering_tire_angle;
  }

  static void convert_to_custom(const ros_message_type & rosidl, custom_type & custom)
  {
    custom.stamp = rosidl.stamp;
    custom.steering_tire_angle = rosidl.steering_tire_angle;
  }
};

#endif  // VEHICLE_CONTROL_INTERFACE__STEERING_STATUS__DEV_HPP_
