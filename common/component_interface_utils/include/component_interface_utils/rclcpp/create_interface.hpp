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

#ifndef COMPONENT_INTERFACE_UTILS__RCLCPP__CREATE_INTERFACE_HPP_
#define COMPONENT_INTERFACE_UTILS__RCLCPP__CREATE_INTERFACE_HPP_

#include <component_interface_utils/rclcpp/service_client.hpp>
#include <component_interface_utils/rclcpp/service_server.hpp>
#include <component_interface_utils/rclcpp/topic_publisher.hpp>
#include <component_interface_utils/rclcpp/topic_subscription.hpp>
#include <component_interface_utils/specs.hpp>
#include <rclcpp/rclcpp.hpp>

#include <utility>

namespace component_interface_utils
{

/// Create a service wrapper for logging. This is a private implementation.
template <class SpecT, class NodeT, class CallbackT>
typename Service<SpecT>::SharedPtr create_service_impl(
  NodeT * node, CallbackT && callback, rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  // Use a node pointer because shared_from_this cannot be used in constructor.
  auto wrapped = Service<SpecT>::wrap(callback, node->get_logger());
  auto service = node->template create_service<typename SpecT::Service>(
    SpecT::name, wrapped, rmw_qos_profile_services_default, group);
  return Service<SpecT>::make_shared(service);
}

/// Create a service wrapper for logging. This is for lambda or bound function.
template <class SpecT, class NodeT, class CallbackT>
typename Service<SpecT>::SharedPtr create_service(
  NodeT * node, CallbackT && callback, rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return create_service_impl<SpecT>(node, std::forward<CallbackT>(callback), group);
}

/// Create a service wrapper for logging. This is for member function of node.
template <class SpecT, class NodeT>
typename Service<SpecT>::SharedPtr create_service(
  NodeT * node, typename Service<SpecT>::template CallbackType<NodeT> callback,
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  return create_service_impl<SpecT>(node, std::bind(callback, node, _1, _2), group);
}

template <class SpecT, class NodeT>
typename rclcpp::Publisher<typename SpecT::Message>::SharedPtr create_publisher(NodeT * node)
{
  return node->template create_publisher<typename SpecT::Message>(SpecT::name, get_qos<SpecT>());
}

}  // namespace component_interface_utils

#endif  // COMPONENT_INTERFACE_UTILS__RCLCPP__CREATE_INTERFACE_HPP_
