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

#include "impl/message.hpp"
#include "impl/service.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>

namespace autoware_interface_library
{

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

}  // namespace autoware_interface_library

#endif  // AUTOWARE_INTERFACE_LIBRARY__AUTOWARE_INTERFACE_LIBRARY_HPP_
