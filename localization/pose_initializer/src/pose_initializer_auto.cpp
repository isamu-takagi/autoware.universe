// Copyright 2020 Autoware Foundation
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

#include "pose_initializer_auto.hpp"

#include <memory>

PoseInitializerAuto::PoseInitializerAuto() : Node("pose_initializer_auto")
{
  /*
  const auto on_state = [this](const State::Message::SharedPtr msg)
  {
    state_ = *msg;
  };
  */

  const auto on_timer = [this]() { RCLCPP_INFO_STREAM(rclcpp::get_logger("node"), this); };
  on_timer();

  // const auto node = component_interface_utils::NodeAdaptor(this);
  // node.init_sub(sub_state_, on_state);
  // node.init_cli(cli_initialize_);

  const auto period = rclcpp::Rate(1.0).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, on_timer);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<PoseInitializerAuto>();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("executor"), "ptr: " << node << " " << node.get());
  executor.add_node(node);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("executor"), "ptr: " << node << " " << node.get());
  executor.spin();
  executor.remove_node(node);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("executor"), "end");
  rclcpp::shutdown();
}
