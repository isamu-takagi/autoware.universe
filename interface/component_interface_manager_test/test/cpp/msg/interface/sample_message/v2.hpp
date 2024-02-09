// Copyright 2024 The Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef CPP__MSG__INTERFACE__SAMPLE_MESSAGE__V2_HPP_
#define CPP__MSG__INTERFACE__SAMPLE_MESSAGE__V2_HPP_

#include <component_interface_manager_cpp/message_traits.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <string>

namespace interface::sample_message::v2
{

struct Vector
{
  double x;
  double y;
  double z;
};

struct Message
{
  Vector velocity;
  double steering;
};

}  // namespace interface::sample_message::v2

namespace component_interface_manager_cpp
{

template <>
struct MessageTraits<interface::sample_message::v2::Message>
{
  using Adaptor =
    rclcpp::TypeAdapter<interface::sample_message::v2::Message, geometry_msgs::msg::Twist>;
  static std::string get_name() { return "/test/message"; }
  static rclcpp::QoS get_pub_qos() { return rclcpp::QoS(1); }
  static rclcpp::QoS get_sub_qos() { return rclcpp::QoS(1); }
};

}  // namespace component_interface_manager_cpp

template <>
struct rclcpp::TypeAdapter<interface::sample_message::v2::Message, geometry_msgs::msg::Twist>
{
  using is_specialized = std::true_type;
  using custom_type = interface::sample_message::v2::Message;
  using ros_message_type = geometry_msgs::msg::Twist;

  static void convert_to_ros_message(const custom_type & custom, ros_message_type & rosmsg)
  {
    rosmsg.linear.x = custom.velocity.x;
    rosmsg.linear.y = custom.velocity.y;
    rosmsg.linear.z = custom.velocity.z;
    rosmsg.angular.z = custom.steering;
  }

  static void convert_to_custom(const ros_message_type & rosmsg, custom_type & custom)
  {
    custom.velocity.x = rosmsg.linear.x;
    custom.velocity.y = rosmsg.linear.y;
    custom.velocity.z = rosmsg.linear.z;
    custom.steering = rosmsg.angular.z;
  }
};

#endif  // CPP__MSG__INTERFACE__SAMPLE_MESSAGE__V2_HPP_
