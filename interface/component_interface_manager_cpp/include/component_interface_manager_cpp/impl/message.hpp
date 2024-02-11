// Copyright 2024 The Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef COMPONENT_INTERFACE_MANAGER_CPP__IMPL__MESSAGE_HPP_
#define COMPONENT_INTERFACE_MANAGER_CPP__IMPL__MESSAGE_HPP_

#include <component_interface_manager_cpp/message_traits.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

namespace component_interface_manager_cpp
{

template <class T>
using Publisher = rclcpp::Publisher<typename MessageTraits<T>::Adaptor>;

template <class T>
using Subscription = rclcpp::Subscription<typename MessageTraits<T>::Adaptor>;

}  // namespace component_interface_manager_cpp

#endif  // COMPONENT_INTERFACE_MANAGER_CPP__IMPL__MESSAGE_HPP_
