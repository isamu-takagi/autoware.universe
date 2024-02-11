// Copyright 2024 The Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef CPP__SRV__INTERFACE__SAMPLE_SERVICE__V2_HPP_
#define CPP__SRV__INTERFACE__SAMPLE_SERVICE__V2_HPP_

#include <component_interface_manager_cpp/service_traits.hpp>
#include <example_interfaces/srv/add_two_ints.hpp>

#include <array>
#include <memory>
#include <string>

namespace interface::sample_service::v2
{

struct ServiceRequest
{
  using SharedPtr = std::shared_ptr<ServiceRequest>;
  std::array<uint64_t, 2> values;
};

struct ServiceResponse
{
  using SharedPtr = std::shared_ptr<ServiceResponse>;
  uint64_t result;
};

struct Service
{
  using Request = ServiceRequest;
  using Response = ServiceResponse;
};

}  // namespace interface::sample_service::v2

namespace component_interface_manager_cpp
{

template <>
struct ServiceTraits<interface::sample_service::v2::Service>
{
  using RosType = example_interfaces::srv::AddTwoInts;
  using Adaptor = rclcpp::TypeAdapter<interface::sample_service::v2::Service, RosType>;
  static std::string get_name() { return "/test/service"; }
  static rclcpp::QoS get_srv_qos() { return rclcpp::ServicesQoS(); }
  static rclcpp::QoS get_cli_qos() { return rclcpp::ServicesQoS(); }
};

}  // namespace component_interface_manager_cpp

template <>
struct rclcpp::TypeAdapter<
  interface::sample_service::v2::Service, example_interfaces::srv::AddTwoInts>
{
  using is_specialized = std::true_type;
  using custom_type = interface::sample_service::v2::Service;
  using ros_message_type = example_interfaces::srv::AddTwoInts;

  static void convert_to_ros_service_request(
    const custom_type::Request & custom, ros_message_type::Request & rosmsg)
  {
    rosmsg.a = custom.values[0];
    rosmsg.b = custom.values[1];
  }

  static void convert_to_custom_service_request(
    const ros_message_type::Request & rosmsg, custom_type::Request & custom)
  {
    custom.values[0] = rosmsg.a;
    custom.values[1] = rosmsg.b;
  }

  static void convert_to_ros_service_response(
    const custom_type::Response & custom, ros_message_type::Response & rosmsg)
  {
    rosmsg.sum = custom.result;
  }

  static void convert_to_custom_service_response(
    const ros_message_type::Response & rosmsg, custom_type::Response & custom)
  {
    custom.result = rosmsg.sum;
  }
};

#endif  // CPP__SRV__INTERFACE__SAMPLE_SERVICE__V2_HPP_
