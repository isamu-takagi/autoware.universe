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

#include "parameter.hpp"

#include <memory>
#include <string>
#include <vector>

namespace default_ad_api
{

ParameterNode::ParameterNode(const rclcpp::NodeOptions & options) : Node("parameter", options)
{
  declare_parameter<std::string>("text", "default value");
  param_handle_ = add_on_set_parameters_callback(
    std::bind(&ParameterNode::on_parameters, this, std::placeholders::_1));
  handle_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  param_handle1_ = handle_->add_parameter_event_callback(
    std::bind(&ParameterNode::on_param_event, this, std::placeholders::_1));
  param_handle2_ = handle_->add_parameter_callback(
    "text", std::bind(&ParameterNode::on_param, this, std::placeholders::_1));
}

void ParameterNode::on_param(const rclcpp::Parameter & parameter)
{
  RCLCPP_INFO_STREAM(get_logger(), "===== param =====");
  RCLCPP_INFO_STREAM(get_logger(), parameter.get_name() << " " << (int)parameter.get_type());
}

void ParameterNode::on_param_event(const rcl_interfaces::msg::ParameterEvent & event)
{
  RCLCPP_INFO_STREAM(get_logger(), "===== event =====");
  RCLCPP_INFO_STREAM(get_logger(), event.node);
  RCLCPP_INFO_STREAM(get_logger(), "[delete]");
  for (const auto & parameter : event.deleted_parameters) {
    RCLCPP_INFO_STREAM(get_logger(), parameter.name << " " << (int)parameter.value.type);
  }
  RCLCPP_INFO_STREAM(get_logger(), "[change]");
  for (const auto & parameter : event.changed_parameters) {
    RCLCPP_INFO_STREAM(get_logger(), parameter.name << " " << (int)parameter.value.type);
  }
  RCLCPP_INFO_STREAM(get_logger(), "[new]");
  for (const auto & parameter : event.new_parameters) {
    RCLCPP_INFO_STREAM(get_logger(), parameter.name << " " << (int)parameter.value.type);
  }
}

rcl_interfaces::msg::SetParametersResult ParameterNode::on_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  RCLCPP_INFO_STREAM(get_logger(), "===== onset =====");
  for (const auto & parameter : parameters) {
    RCLCPP_INFO_STREAM(get_logger(), parameter.get_name() << " " << (int)parameter.get_type());
    // parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
    // parameter.as_string();
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "success";
  return result;
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::ParameterNode)
