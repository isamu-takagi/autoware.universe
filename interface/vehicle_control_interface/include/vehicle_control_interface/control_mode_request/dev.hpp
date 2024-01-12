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

#ifndef VEHICLE_CONTROL_INTERFACE__CONTROL_MODE_REQUEST__DEV_HPP_
#define VEHICLE_CONTROL_INTERFACE__CONTROL_MODE_REQUEST__DEV_HPP_

#include <rclcpp/qos.hpp>
#include <rclcpp/type_adapter.hpp>

#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>

#include <memory>
#include <string>

namespace vehicle_control_interface::control_mode_request::dev
{

struct CustomService
{
  enum class Mode {
    NO_COMMAND,
    AUTONOMOUS,
    AUTONOMOUS_STEER_ONLY,
    AUTONOMOUS_VELOCITY_ONLY,
    MANUAL
  };

  struct Request
  {
    using SharedPtr = std::shared_ptr<Request>;
    builtin_interfaces::msg::Time stamp;
    Mode mode2;
  };

  struct Response
  {
    using SharedPtr = std::shared_ptr<Response>;
    bool success2;
  };
};

struct Interface
{
  using RosType = autoware_vehicle_msgs::srv::ControlModeCommand;
  using Service = CustomService;
  using Adaptor = rclcpp::TypeAdapter<Service, RosType>;

  using Request = typename Service::Request;
  using Response = typename Service::Response;

  static std::string get_name() { return "/service/name"; }
  static rclcpp::QoS get_srv_qos() { return rclcpp::ServicesQoS(); }
  static rclcpp::QoS get_cli_qos() { return rclcpp::ServicesQoS(); }
};

using CustomMode = CustomService::Mode;
using RosidlMode = autoware_vehicle_msgs::srv::ControlModeCommand::Request::_mode_type;

CustomMode convert_to_custom_mode(const RosidlMode & mode)
{
  return CustomMode::NO_COMMAND;
}

RosidlMode convert_to_rosidl_mode(const CustomMode & mode)
{
  return autoware_vehicle_msgs::srv::ControlModeCommand::Request::NO_COMMAND;
}

}  // namespace vehicle_control_interface::control_mode_request::dev

template <>
struct rclcpp::TypeAdapter<
  vehicle_control_interface::control_mode_request::dev::CustomService,
  autoware_vehicle_msgs::srv::ControlModeCommand>
{
  using is_specialized = std::true_type;
  using custom_type = vehicle_control_interface::control_mode_request::dev::CustomService;
  using ros_message_type = autoware_vehicle_msgs::srv::ControlModeCommand;

  static void convert_to_ros_service_request(
    const custom_type::Request & custom, ros_message_type::Request & rosidl)
  {
    using vehicle_control_interface::control_mode_request::dev::convert_to_rosidl_mode;
    rosidl.stamp = custom.stamp;
    rosidl.mode = convert_to_rosidl_mode(custom.mode2);
  }

  static void convert_to_custom_service_request(
    const ros_message_type::Request & rosidl, custom_type::Request & custom)
  {
    using vehicle_control_interface::control_mode_request::dev::convert_to_custom_mode;
    custom.stamp = rosidl.stamp;
    custom.mode2 = convert_to_custom_mode(rosidl.mode);
  }

  static void convert_to_ros_service_response(
    const custom_type::Response & custom, ros_message_type::Response & rosidl)
  {
    rosidl.success = custom.success2;
  }

  static void convert_to_custom_service_response(
    const ros_message_type::Response & rosidl, custom_type::Response & custom)
  {
    custom.success2 = rosidl.success;
  }
};

#endif  // VEHICLE_CONTROL_INTERFACE__CONTROL_MODE_REQUEST__DEV_HPP_
