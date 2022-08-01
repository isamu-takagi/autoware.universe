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

#ifndef TEST_HPP_
#define TEST_HPP_

#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

struct SetBool
{
  using Service = std_srvs::srv::SetBool;
  static constexpr char name[] = "/api/test";
};

namespace default_ad_api
{

class TestNode : public rclcpp::Node
{
public:
  explicit TestNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::CallbackGroup::SharedPtr group_srv_;
  component_interface_utils::Service<SetBool>::SharedPtr srv_;
  component_interface_utils::Client<SetBool>::SharedPtr cli_;
};

}  // namespace default_ad_api

#endif  // TEST_HPP_
