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

#include "loader.hpp"

namespace v2x_gate_driver
{

Loader::Loader(const rclcpp::NodeOptions & options) : Node("v2x_gate_driver", options)
{
  srv_acquire_ = create_service<AcquireGateLock>(
    "~/srv/acquire",
    std::bind(&Loader::on_acquire, this, std::placeholders::_1, std::placeholders::_2));
}

void Loader::on_acquire(
  const AcquireGateLock::Request::SharedPtr req, const AcquireGateLock::Response::SharedPtr res)
{
  (void)req;
  (void)res;
  RCLCPP_INFO_STREAM(get_logger(), "on_acquire");
}

}  // namespace v2x_gate_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(v2x_gate_driver::Loader)
