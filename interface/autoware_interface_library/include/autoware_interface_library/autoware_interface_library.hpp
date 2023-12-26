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

#ifndef AUTOWARE_INTERFACE_LIBRARY__AUTOWARE_INTERFACE_LIBRARY_HPP_
#define AUTOWARE_INTERFACE_LIBRARY__AUTOWARE_INTERFACE_LIBRARY_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>

namespace autoware_interface_library
{

template <class T>
using Publisher = rclcpp::Publisher<typename T::Adaptor>;

template <class T>
using Subscription = rclcpp::Subscription<typename T::Adaptor>;

template <class T>
using Client = rclcpp::Client<typename T::Adaptor>;

template <class T>
using Service = rclcpp::Service<typename T::Adaptor>;

class AutowareInterfaceAdaptor
{
public:
  explicit AutowareInterfaceAdaptor(rclcpp::Node * node) : node_(node) {}

  template <class T, class... Args>
  auto create_subscription(Args &&... args)
  {
    const auto name = T::get_name();
    const auto qos = T::get_sub_qos();
    return node_->create_subscription<typename T::Adaptor>(name, qos, std::forward<Args>(args)...);
  }

  template <class T, class... Args>
  auto create_publisher(Args &&... args)
  {
    const auto name = T::get_name();
    const auto qos = T::get_pub_qos();
    return node_->create_publisher<typename T::Adaptor>(name, qos, std::forward<Args>(args)...);
  }

  template <class T, class... Args>
  auto create_subscription(const std::string & name, const rclcpp::QoS & qos, Args &&... args)
  {
    return node_->create_subscription<typename T::Adaptor>(name, qos, std::forward<Args>(args)...);
  }

  template <class T, class... Args>
  auto create_publisher(const std::string & name, const rclcpp::QoS & qos, Args &&... args)
  {
    return node_->create_publisher<typename T::Adaptor>(name, qos, std::forward<Args>(args)...);
  }

private:
  rclcpp::Node * node_;
};

}  // namespace autoware_interface_library

#endif  // AUTOWARE_INTERFACE_LIBRARY__AUTOWARE_INTERFACE_LIBRARY_HPP_
