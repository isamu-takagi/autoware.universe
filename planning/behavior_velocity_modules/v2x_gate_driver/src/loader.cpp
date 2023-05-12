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

#include <string>
#include <vector>

std::string join(const std::vector<std::string> & ids)
{
  std::string result;
  for (const auto & id : ids) {
    result += id + " ";
  }
  return result;
}

namespace v2x_gate_driver
{

GateLockServerStatusArray convert(const GateLockClientStatusArray & in)
{
  GateLockServerStatusArray out;
  for (const auto & client : in.statuses) {
    tier4_v2x_msgs::msg::GateLockServerStatus server;
    server.target = client.target;
    out.statuses.push_back(server);
  }
  return out;
}

Loader::Loader(const rclcpp::NodeOptions & options) : Node("v2x_gate_driver", options)
{
  sub_update_ = create_subscription<GateLockClientStatusArray>(
    "~/sub/update", rclcpp::QoS(1), std::bind(&Loader::on_status, this, std::placeholders::_1));

  pub_status_ = create_publisher<GateLockServerStatusArray>("~/pub/status", rclcpp::QoS(1));
}

void Loader::on_status(const GateLockClientStatusArray::ConstSharedPtr msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "on_status");
  for (const auto & status : msg->statuses) {
    const auto & t = status.target;
    RCLCPP_INFO_STREAM(get_logger(), " - " << t.category << " " << t.area << " " << t.sequence);
  }

  temp_.push_back(convert(*msg));
  if (30 < temp_.size()) {
    pub_status_->publish(temp_.front());
    temp_.pop_front();
  }
}

}  // namespace v2x_gate_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(v2x_gate_driver::Loader)
