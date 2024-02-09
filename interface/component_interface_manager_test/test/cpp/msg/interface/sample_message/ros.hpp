// Copyright 2024 The Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef CPP__MSG__INTERFACE__SAMPLE_MESSAGE__ROS_HPP_
#define CPP__MSG__INTERFACE__SAMPLE_MESSAGE__ROS_HPP_

#include <component_interface_manager_cpp/message_traits.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <string>

namespace interface::sample_message::ros
{

using Message = geometry_msgs::msg::Twist;

}  // namespace interface::sample_message::ros

namespace component_interface_manager_cpp
{

template <>
struct MessageTraits<interface::sample_message::ros::Message>
{
  using Adaptor = interface::sample_message::ros::Message;
  static std::string get_name() { return "/test/message"; }
  static rclcpp::QoS get_pub_qos() { return rclcpp::QoS(1); }
  static rclcpp::QoS get_sub_qos() { return rclcpp::QoS(1); }
};

}  // namespace component_interface_manager_cpp

#endif  // CPP__MSG__INTERFACE__SAMPLE_MESSAGE__ROS_HPP_
