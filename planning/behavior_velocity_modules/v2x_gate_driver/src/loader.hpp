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

#ifndef LOADER_HPP_
#define LOADER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_v2x_msgs/msg/gate_lock_client_status_array.hpp>
#include <tier4_v2x_msgs/msg/gate_lock_server_status_array.hpp>

#include <deque>

namespace v2x_gate_driver
{

using tier4_v2x_msgs::msg::GateLockClientStatusArray;
using tier4_v2x_msgs::msg::GateLockServerStatusArray;

class Loader : public rclcpp::Node
{
public:
  explicit Loader(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<GateLockClientStatusArray>::SharedPtr sub_update_;
  rclcpp::Publisher<GateLockServerStatusArray>::SharedPtr pub_status_;
  void on_status(const GateLockClientStatusArray::ConstSharedPtr msg);

  std::deque<GateLockServerStatusArray> temp_;
};

}  // namespace v2x_gate_driver

#endif  // LOADER_HPP_
