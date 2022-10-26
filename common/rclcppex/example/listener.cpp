// Copyright 2022 Takagi, Isamu
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

#include "listener.hpp"

#include <memory>

Listener::Listener() : Node("listener")
{
  const auto callback = [this](const std_msgs::msg::String::ConstSharedPtr msg) {
    RCLCPP_INFO_STREAM(get_logger(), "Msg : " << msg->data);
  };
  sub_ = create_subscription<std_msgs::msg::String>("/message", rclcpp::QoS(1), callback);

  const auto rate = rclcpp::Rate(1.0);
  timer_ = rclcpp::create_timer(this, get_clock(), rate.period(), [this]() {
    RCLCPP_INFO_STREAM(get_logger(), "Rate: " << sub_->get_rate());
  });
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = std::make_shared<Listener>();
  exec.add_node(node);
  exec.spin();
  exec.remove_node(node);
  rclcpp::shutdown();
}
