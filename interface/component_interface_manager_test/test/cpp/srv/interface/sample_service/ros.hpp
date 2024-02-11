// Copyright 2024 The Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef CPP__SRV__INTERFACE__SAMPLE_SERVICE__ROS_HPP_
#define CPP__SRV__INTERFACE__SAMPLE_SERVICE__ROS_HPP_

#include <component_interface_manager_cpp/service_traits.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

#include <memory>
#include <string>

namespace interface::sample_service::ros
{

using Service = example_interfaces::srv::AddTwoInts;

}  // namespace interface::sample_service::ros

namespace component_interface_manager_cpp
{

struct DummyAdaptor
{
  using is_specialized = std::false_type;
};

template <>
struct ServiceTraits<interface::sample_service::ros::Service>
{
  using RosType = example_interfaces::srv::AddTwoInts;
  using Adaptor = DummyAdaptor;
  static std::string get_name() { return "/test/service"; }
  static rclcpp::QoS get_srv_qos() { return rclcpp::ServicesQoS(); }
  static rclcpp::QoS get_cli_qos() { return rclcpp::ServicesQoS(); }
};

}  // namespace component_interface_manager_cpp

#endif  // CPP__SRV__INTERFACE__SAMPLE_SERVICE__ROS_HPP_
