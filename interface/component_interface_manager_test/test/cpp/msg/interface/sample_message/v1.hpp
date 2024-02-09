// Copyright 2024 The Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef CPP__MSG__INTERFACE__SAMPLE_MESSAGE__V1_HPP_
#define CPP__MSG__INTERFACE__SAMPLE_MESSAGE__V1_HPP_

#include <component_interface_manager_cpp/message_traits.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <string>

namespace interface::sample_message::v1
{

struct Message
{
  double velocity;
  double steering;
};

}  // namespace interface::sample_message::v1

namespace component_interface_manager_cpp
{

template <>
struct MessageTraits<interface::sample_message::v1::Message>
{
  using Adaptor =
    rclcpp::TypeAdapter<interface::sample_message::v1::Message, geometry_msgs::msg::Twist>;
  static std::string get_name() { return "/test/message"; }
  static rclcpp::QoS get_pub_qos() { return rclcpp::QoS(1); }
  static rclcpp::QoS get_sub_qos() { return rclcpp::QoS(1); }
};

}  // namespace component_interface_manager_cpp

template <>
struct rclcpp::TypeAdapter<interface::sample_message::v1::Message, geometry_msgs::msg::Twist>
{
  using is_specialized = std::true_type;
  using custom_type = interface::sample_message::v1::Message;
  using ros_message_type = geometry_msgs::msg::Twist;

  static void convert_to_ros_message(const custom_type & custom, ros_message_type & rosmsg)
  {
    rosmsg.linear.x = custom.velocity;
    rosmsg.angular.z = custom.steering;
  }

  static void convert_to_custom(const ros_message_type & rosmsg, custom_type & custom)
  {
    custom.velocity = rosmsg.linear.x;
    custom.steering = rosmsg.angular.z;
  }
};

#endif  // CPP__MSG__INTERFACE__SAMPLE_MESSAGE__V1_HPP_
