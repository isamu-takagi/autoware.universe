// Copyright 2024 The Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef COMPONENT_INTERFACE_MANAGER_CPP__COMPONENT_INTERFACE_MANAGER_CPP_HPP_
#define COMPONENT_INTERFACE_MANAGER_CPP__COMPONENT_INTERFACE_MANAGER_CPP_HPP_

#include "impl/message.hpp"
#include "impl/service.hpp"

#include <component_interface_manager_cpp/message_traits.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>

namespace component_interface_manager_cpp
{

class ComponentInterfaceManager
{
public:
  explicit ComponentInterfaceManager(rclcpp::Node * node) : node_(node) {}

  template <class T, class... Args>
  auto create_subscription(const std::string & name, const rclcpp::QoS & qos, Args &&... args)
  {
    return node_->create_subscription<typename T::Adaptor>(name, qos, std::forward<Args>(args)...);
  }

  template <class T, class... Args>
  auto create_subscription(Args &&... args)
  {
    using Traits = MessageTraits<T>;
    const auto name = Traits::get_name();
    const auto qos = Traits::get_sub_qos();
    return node_->create_subscription<typename Traits::Adaptor>(
      name, qos, std::forward<Args>(args)...);
  }

  template <class T, class... Args>
  auto create_publisher(const std::string & name, const rclcpp::QoS & qos, Args &&... args)
  {
    return node_->create_publisher<typename T::Adaptor>(name, qos, std::forward<Args>(args)...);
  }

  template <class T, class... Args>
  auto create_publisher(Args &&... args)
  {
    using Traits = MessageTraits<T>;
    const auto name = Traits::get_name();
    const auto qos = Traits::get_pub_qos();
    return node_->create_publisher<typename Traits::Adaptor>(
      name, qos, std::forward<Args>(args)...);
  }

  template <class T, class... Args>
  auto create_client(Args &&... args)
  {
    return std::make_shared<Client<T>>(node_, std::forward<Args>(args)...);
  }

  template <class T, class C>
  auto create_service(C && callback)
  {
    return std::make_shared<Service<T>>(node_, std::forward<C>(callback));
  }

private:
  rclcpp::Node * node_;
};

}  // namespace component_interface_manager_cpp

#endif  // COMPONENT_INTERFACE_MANAGER_CPP__COMPONENT_INTERFACE_MANAGER_CPP_HPP_
