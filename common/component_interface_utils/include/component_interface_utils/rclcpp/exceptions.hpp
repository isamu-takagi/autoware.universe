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

#ifndef COMPONENT_INTERFACE_UTILS__RCLCPP__EXCEPTIONS_HPP_
#define COMPONENT_INTERFACE_UTILS__RCLCPP__EXCEPTIONS_HPP_

#include <autoware_ad_api_msgs/msg/response_status.hpp>

#include <stdexcept>
#include <string>

namespace component_interface_utils
{

class ServiceException : public std::runtime_error
{
public:
  using ResponseStatus = autoware_ad_api_msgs::msg::ResponseStatus;
  using ResponseStatusCode = ResponseStatus::_code_type;

  ServiceException(const std::string & message, ResponseStatusCode code)
  : std::runtime_error(message)
  {
    code_ = code;
  }
  ResponseStatusCode code() const { return code_; }

private:
  ResponseStatusCode code_;
};

class ServiceUnready : public ServiceException
{
  explicit ServiceUnready(const std::string & message)
  : ServiceException(message, ResponseStatus::SERVICE_UNREADY)
  {
  }
};

class ServiceTimeout : public ServiceException
{
  explicit ServiceTimeout(const std::string & message)
  : ServiceException(message, ResponseStatus::SERVICE_TIMEOUT)
  {
  }
};

}  // namespace component_interface_utils

#endif  // COMPONENT_INTERFACE_UTILS__RCLCPP__EXCEPTIONS_HPP_
