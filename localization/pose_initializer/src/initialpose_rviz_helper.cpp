// Copyright 2022 Autoware Foundation
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

#include "initialpose_rviz_helper.hpp"

#include <memory>

InitialPoseRvizHelper::InitialPoseRvizHelper() : Node("initial_pose_rviz_helper")
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  const auto node = component_interface_utils::NodeAdaptor(this);
  node.init_cli(cli_initialize_);

  auto on_initial_pose = std::bind(&InitialPoseRvizHelper::OnInitialPose, this, _1);
  sub_initial_pose_ =
    create_subscription<PoseWithCovarianceStamped>("initialpose", rclcpp::QoS(1), on_initial_pose);
}

void InitialPoseRvizHelper::OnInitialPose(PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  try {
    const auto req = std::make_shared<Initialize::Service::Request>();
    req->pose.push_back(*msg);
    cli_initialize_->async_send_request(req);
  } catch (const component_interface_utils::ServiceException & error) {
    RCLCPP_ERROR_STREAM(get_logger(), error.what());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<InitialPoseRvizHelper>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
