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

#ifndef PARAMETER_HPP_
#define PARAMETER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace default_ad_api
{

class ParameterNode : public rclcpp::Node
{
public:
  explicit ParameterNode(const rclcpp::NodeOptions & options);

private:
  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter> & parameters);
  void on_param(const rclcpp::Parameter & event);
  void on_param_event(const rcl_interfaces::msg::ParameterEvent & event);
  OnSetParametersCallbackHandle::SharedPtr param_handle_;
  std::shared_ptr<rclcpp::ParameterEventHandler> handle_;
  rclcpp::ParameterEventCallbackHandle::SharedPtr param_handle1_;
  rclcpp::ParameterCallbackHandle::SharedPtr param_handle2_;
};

}  // namespace default_ad_api

#endif  // PARAMETER_HPP_
