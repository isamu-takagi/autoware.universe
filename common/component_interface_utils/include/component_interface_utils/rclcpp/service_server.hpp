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

#ifndef COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_SERVER_HPP_
#define COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_SERVER_HPP_

#include <component_interface_utils/rclcpp/exceptions.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/service.hpp>

#include <autoware_ad_api_msgs/msg/response_status.hpp>

namespace component_interface_utils
{

/// The wrapper class of rclcpp::Service for logging.
template <class SpecT>
class Service
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Service)
  using SpecType = SpecT;
  using WrapType = rclcpp::Service<typename SpecT::Service>;

  template <class, template <class> class, class = std::void_t<>>
  struct detect : std::false_type
  {
  };
  template <class T, template <class> class Check>
  struct detect<T, Check, std::void_t<Check<T>>> : std::true_type
  {
  };
  template <class T>
  using has_status_impl = decltype(std::declval<T>().status);
  template <class T>
  using has_status_type = detect<T, has_status_impl>;

  /// Constructor.
  explicit Service(typename WrapType::SharedPtr service)
  {
    service_ = service;  // to keep the reference count
  }

  /// Create a service callback with logging added.
  template <class CallbackT>
  static auto wrap(CallbackT && callback, const rclcpp::Logger & logger)
  {
    auto wrapped = [logger, callback](
                     typename SpecT::Service::Request::SharedPtr request,
                     typename SpecT::Service::Response::SharedPtr response) {
#ifdef ROS_DISTRO_GALACTIC
      using rosidl_generator_traits::to_yaml;
#endif
      RCLCPP_INFO_STREAM(logger, "service call: " << SpecT::name << "\n" << to_yaml(*request));
      if constexpr (!has_status_type<typename SpecT::Service::Response>::value) {
        callback(request, response);
      } else {
        using ResponseStatus = autoware_ad_api_msgs::msg::ResponseStatus;
        try {
          callback(request, response);
        } catch (const ServiceUnready & error) {
          response->status.level = ResponseStatus::ERROR;
          response->status.code = ResponseStatus::INTERNAL_SERVICE_UNREADY;
          response->status.message = error.what();
        } catch (const ServiceTimeout & error) {
          response->status.level = ResponseStatus::ERROR;
          response->status.code = ResponseStatus::INTERNAL_SERVICE_TIMEOUT;
          response->status.message = error.what();
        }
      }
      RCLCPP_INFO_STREAM(logger, "service exit: " << SpecT::name << "\n" << to_yaml(*response));
    };
    return wrapped;
  }

private:
  RCLCPP_DISABLE_COPY(Service)
  typename WrapType::SharedPtr service_;
};

}  // namespace component_interface_utils

#endif  // COMPONENT_INTERFACE_UTILS__RCLCPP__SERVICE_SERVER_HPP_
