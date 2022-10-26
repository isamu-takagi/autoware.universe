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

#include "talker.hpp"

#include <memory>

Talker::Talker() : Node("talker")
{
  pub_ = create_publisher<std_msgs::msg::String>("/message", rclcpp::QoS(1));

  const auto rate = rclcpp::Rate(2.0);
  timer_ = rclcpp::create_timer(this, get_clock(), rate.period(), [this]() {
    std_msgs::msg::String msg;
    msg.data = "Message " + std::to_string(++count_);
    pub_->publish(msg);
    RCLCPP_INFO_STREAM(get_logger(), "Msg : " << msg.data);
    RCLCPP_INFO_STREAM(get_logger(), "Rate: " << pub_->get_rate());
  });
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = std::make_shared<Talker>();
  exec.add_node(node);
  exec.spin();
  exec.remove_node(node);
  rclcpp::shutdown();
}
