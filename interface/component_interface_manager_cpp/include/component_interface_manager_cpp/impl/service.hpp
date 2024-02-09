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

#ifndef COMPONENT_INTERFACE_MANAGER_CPP__IMPL__SERVICE_HPP_
#define COMPONENT_INTERFACE_MANAGER_CPP__IMPL__SERVICE_HPP_

#include <rclcpp/client.hpp>
#include <rclcpp/service.hpp>

#include <memory>
#include <utility>

namespace component_interface_manager_cpp
{

namespace detail
{

template <typename T>
struct FutureAndRequestId
{
  using Future = std::shared_future<typename T::Adaptor::ros_message_type::Response::SharedPtr>;
  using SharedResponse = std::shared_ptr<typename T::Response>;

  Future future;
  int64_t request_id;

  FutureAndRequestId(Future && future, int64_t req_id) : future(future), request_id(req_id) {}

  SharedResponse get()
  {
    SharedResponse custom = std::make_shared<typename T::Response>();
    T::Adaptor::convert_to_custom_service_response(*future.get(), *custom);
    return custom;
  }
};

}  // namespace detail

template <class T>
class Client
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Client)
  using RosType = typename T::Adaptor::ros_message_type;
  using RosClient = rclcpp::Client<RosType>;
  using SharedRequest = std::shared_ptr<typename T::Request>;
  using SharedResponse = std::shared_ptr<typename T::Response>;
  using SharedFutureAndRequestId = detail::FutureAndRequestId<T>;

  explicit Client(typename RosClient::SharedPtr client) { client_ = client; }

  /*
  auto async_send_request(SharedRequest custom)
  {
    const auto rosidl = std::make_shared<typename RosType::Request>();
    T::Adaptor::convert_to_ros_service_request(*custom, *rosidl);
    return client_->async_send_request(rosidl);
  }
  */

  template <class CallbackT>
  auto async_send_request(SharedRequest request, CallbackT && callback)
  {
    auto wrapper = [callback](typename RosClient::SharedFuture future) {
      SharedResponse custom = std::make_shared<typename T::Response>();
      T::Adaptor::convert_to_custom_service_response(*future.get(), *custom);
      callback(custom);
    };

    auto rosidl = std::make_shared<typename RosType::Request>();
    T::Adaptor::convert_to_ros_service_request(*request, *rosidl);

    auto result = client_->async_send_request(rosidl, wrapper);
    return SharedFutureAndRequestId(std::move(result.future), result.request_id);
  }

private:
  typename RosClient::SharedPtr client_;
};

template <class T>
class Service
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Service)
  using RosType = typename T::Adaptor::ros_message_type;
  using RosService = rclcpp::Service<RosType>;

  explicit Service(typename RosService::SharedPtr service) { service_ = service; }

private:
  typename RosService::SharedPtr service_;
};

}  // namespace component_interface_manager_cpp

#endif  // COMPONENT_INTERFACE_MANAGER_CPP__IMPL__SERVICE_HPP_
