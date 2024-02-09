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
  auto create_subscription(Args &&... args)
  {
    using Traits = MessageTraits<T>;
    const auto name = Traits::get_name();
    const auto qos = Traits::get_sub_qos();
    return node_->create_subscription<typename Traits::Adaptor>(
      name, qos, std::forward<Args>(args)...);
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
    using RosType = typename T::Adaptor::ros_message_type;
    const auto name = T::get_name();
    const auto qos = T::get_cli_qos().get_rmw_qos_profile();
    const auto client = node_->create_client<RosType>(name, qos, std::forward<Args>(args)...);
    return std::make_shared<Client<T>>(client);
  }

  template <class T, class C>
  auto create_service(C && callback)
  {
    auto wrapper = [callback](
                     typename T::RosType::Request::SharedPtr req,
                     typename T::RosType::Response::SharedPtr res) {
      const auto custom_req = std::make_shared<typename T::Request>();
      const auto custom_res = std::make_shared<typename T::Response>();
      T::Adaptor::convert_to_custom_service_request(*req, *custom_req);
      callback(custom_req, custom_res);
      T::Adaptor::convert_to_ros_service_response(*custom_res, *res);
    };

    using RosType = typename T::Adaptor::ros_message_type;
    const auto name = T::get_name();
    const auto qos = T::get_srv_qos().get_rmw_qos_profile();
    const auto service = node_->create_service<RosType>(name, wrapper, qos);
    return std::make_shared<Service<T>>(service);
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

}  // namespace component_interface_manager_cpp

#endif  // COMPONENT_INTERFACE_MANAGER_CPP__COMPONENT_INTERFACE_MANAGER_CPP_HPP_
