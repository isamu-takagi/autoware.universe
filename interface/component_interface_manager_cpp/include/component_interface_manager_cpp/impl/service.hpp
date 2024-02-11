// Copyright 2024 The Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef COMPONENT_INTERFACE_MANAGER_CPP__IMPL__SERVICE_HPP_
#define COMPONENT_INTERFACE_MANAGER_CPP__IMPL__SERVICE_HPP_

#include <component_interface_manager_cpp/service_traits.hpp>
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
  using Future = std::shared_future<typename ServiceTraits<T>::RosType::Response::SharedPtr>;
  using SharedResponse = std::shared_ptr<typename T::Response>;

  Future future;
  int64_t request_id;

  FutureAndRequestId(Future && future, int64_t req_id) : future(future), request_id(req_id) {}

  SharedResponse get()
  {
    SharedResponse custom = std::make_shared<typename T::Response>();
    ServiceTraits<T>::Adaptor::convert_to_custom_service_response(*future.get(), *custom);
    return custom;
  }
};

}  // namespace detail

template <class T>
class Client
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Client)
  using RosType = typename ServiceTraits<T>::RosType;
  using RosClient = rclcpp::Client<RosType>;
  using SharedRequest = std::shared_ptr<typename T::Request>;
  using SharedResponse = std::shared_ptr<typename T::Response>;
  using SharedFutureAndRequestId = detail::FutureAndRequestId<T>;

  template <class N, class... Args>
  explicit Client(N & node, Args &&... args)
  {
    using Traits = ServiceTraits<T>;
    const auto name = Traits::get_name();
    const auto qos = Traits::get_cli_qos().get_rmw_qos_profile();
    client_ = node->template create_client<RosType>(name, qos, std::forward<Args>(args)...);
  }

  auto async_send_request(SharedRequest request)
  {
    using Traits = ServiceTraits<T>;

    if constexpr (Traits::Adaptor::is_specialized::value) {
      auto rosmsg = std::make_shared<typename RosType::Request>();
      Traits::Adaptor::convert_to_ros_service_request(*request, *rosmsg);

      auto result = client_->async_send_request(rosmsg);
      return SharedFutureAndRequestId(std::move(result.future), result.request_id);
    } else {
      return client_->async_send_request(request);
    }
  }

  template <class CallbackT>
  auto async_send_request(SharedRequest request, CallbackT && callback)
  {
    using Traits = ServiceTraits<T>;

    if constexpr (Traits::Adaptor::is_specialized::value) {
      auto wrapper = [callback](typename RosClient::SharedFuture future) {
        SharedResponse custom = std::make_shared<typename T::Response>();
        Traits::Adaptor::convert_to_custom_service_response(*future.get(), *custom);
        callback(custom);
      };

      auto rosmsg = std::make_shared<typename RosType::Request>();
      Traits::Adaptor::convert_to_ros_service_request(*request, *rosmsg);

      auto result = client_->async_send_request(rosmsg, wrapper);
      return SharedFutureAndRequestId(std::move(result.future), result.request_id);
    } else {
      auto wrapper = [callback](typename RosClient::SharedFuture future) {
        callback(future.get());
      };
      return client_->async_send_request(request, wrapper);
    }
  }

private:
  typename RosClient::SharedPtr client_;
};

template <class T>
class Service
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Service)
  using RosType = typename ServiceTraits<T>::RosType;
  using RosService = rclcpp::Service<RosType>;
  using SharedRequest = std::shared_ptr<typename T::Request>;
  using SharedResponse = std::shared_ptr<typename T::Response>;

  template <class N, class C>
  Service(N & node, C && callback)
  {
    using Traits = ServiceTraits<T>;
    using std::placeholders::_1;
    using std::placeholders::_2;
    const auto name = Traits::get_name();
    const auto qos = Traits::get_srv_qos().get_rmw_qos_profile();
    typename RosService::CallbackType func = std::bind(&Service<T>::on_service, this, _1, _2);
    service_ = node->template create_service<RosType>(name, func, qos);
    callback_ = callback;
  }

protected:
  void on_service(
    typename RosType::Request::SharedPtr req, typename RosType::Response::SharedPtr res)
  {
    using Traits = ServiceTraits<T>;

    if constexpr (Traits::Adaptor::is_specialized::value) {
      const auto custom_req = std::make_shared<typename T::Request>();
      const auto custom_res = std::make_shared<typename T::Response>();
      Traits::Adaptor::convert_to_custom_service_request(*req, *custom_req);
      callback_(custom_req, custom_res);
      Traits::Adaptor::convert_to_ros_service_response(*custom_res, *res);
    } else {
      callback_(req, res);
    }
  };

private:
  typename RosService::SharedPtr service_;
  std::function<void(const SharedRequest, SharedResponse)> callback_;
};

}  // namespace component_interface_manager_cpp

#endif  // COMPONENT_INTERFACE_MANAGER_CPP__IMPL__SERVICE_HPP_
