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

#ifndef COMPONENT_INTERFACE_UTILS__RCLCPP_HPP_
#define COMPONENT_INTERFACE_UTILS__RCLCPP_HPP_

#include <component_interface_utils/rclcpp/create_interface.hpp>

#include <utility>

namespace component_interface_utils
{

class NodeAdaptor
{
private:
  using CallbackGroup = rclcpp::CallbackGroup::SharedPtr;

public:
  /// Constructor.
  explicit NodeAdaptor(rclcpp::Node * node) : node_(node) {}

  /// Create a client wrapper for logging. This is for member function of node.
  template <class SharedPtrT>
  void init_client(SharedPtrT & cli, CallbackGroup group = nullptr) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    cli = create_client_impl<SpecT>(node_, group);
  }

  /// Create a service wrapper for logging.
  template <class SharedPtrT, class CallbackT>
  void init_service(SharedPtrT & srv, CallbackT && callback, CallbackGroup group = nullptr) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    srv = create_service_impl<SpecT>(node_, std::forward<CallbackT>(callback), group);
  }

  /// Create a publisher using traits like services.
  template <class SharedPtrT>
  void init_publisher(SharedPtrT & pub) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    pub = create_publisher_impl<SpecT>(node_);
  }

  /// Create a subscription using traits like services.
  template <class SharedPtrT, class CallbackT>
  void init_subscription(SharedPtrT & sub, CallbackT && callback) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    sub = create_subscription_impl<SpecT>(node_, std::forward<CallbackT>(callback));
  }

private:
  // Use a node pointer because shared_from_this cannot be used in constructor.
  rclcpp::Node * node_;
};

}  // namespace component_interface_utils

#endif  // COMPONENT_INTERFACE_UTILS__RCLCPP_HPP_
