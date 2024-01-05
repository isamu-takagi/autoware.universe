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

#ifndef AUTOWARE_INTERFACE_LIBRARY__IMPL__SERVICE_HPP_
#define AUTOWARE_INTERFACE_LIBRARY__IMPL__SERVICE_HPP_

#include <rclcpp/client.hpp>
#include <rclcpp/service.hpp>

#include <memory>
#include <utility>

namespace autoware_interface_library
{

template <class T>
class Client
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Client)
  using RosType = typename T::Adaptor::ros_message_type;
  using WrapType = rclcpp::Client<RosType>;

  using SharedRequest = std::shared_ptr<typename T::Request>;
  using SharedResponse = std::shared_ptr<typename T::Response>;

  using SharedFuture = std::shared_future<SharedResponse>;

  explicit Client(typename WrapType::SharedPtr client) { client_ = client; }

  auto async_send_request(SharedRequest custom)
  {
    const auto rosidl = std::make_shared<typename RosType::Request>();
    T::Adaptor::convert_to_ros_service_request(*custom, *rosidl);
    return client_->async_send_request(rosidl);
  }

  template <class... Args>
  auto async_send_request(SharedRequest custom, Args &&... args)
  {
    const auto rosidl = std::make_shared<typename RosType::Request>();
    T::Adaptor::convert_to_ros_service_request(*custom, *rosidl);

    const auto wrapper = [](WrapType::SharedFuture future) {};
    return client_->async_send_request(rosidl, std::forward<Args>(args)...);
  }

private:
  typename WrapType::SharedPtr client_;
};

template <class T>
class Service
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Service)
};

}  // namespace autoware_interface_library

#endif  // AUTOWARE_INTERFACE_LIBRARY__IMPL__SERVICE_HPP_
