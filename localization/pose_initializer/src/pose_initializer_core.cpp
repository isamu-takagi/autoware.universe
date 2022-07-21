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

#include "pose_initializer_core.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

PoseInitializer::PoseInitializer() : Node("pose_initializer")
{
  const auto node = component_interface_utils::NodeAdaptor(this);
  using std::placeholders::_1;
  using std::placeholders::_2;

  const auto on_initialize = std::bind(&PoseInitializer::OnInitialize, this, _1, _2);
  node.init_pub(pub_state_);
  node.init_srv(srv_initialize_, on_initialize);
}

void PoseInitializer::OnInitialize(ROS_SERVICE_ARG(Initialize, res, req))
{
  (void)res;
  (void)req;
}

/*
const auto node = component_interface_utils::NodeAdaptor(this);
node.init_srv(srv_driving_engage_, on_driving_engage);
node.init_pub(pub_driving_state_);
node.init_sub(sub_autoware_state_, on_autoware_state);

bool PoseInitializer::callAlignServiceAndPublishResult(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr input_pose_msg)
{
  if (request_id_ != response_id_) {
    RCLCPP_ERROR(get_logger(), "Did not receive response for previous NDT Align Server call");
    return false;
  }
  auto req = std::make_shared<tier4_localization_msgs::srv::PoseWithCovarianceStamped::Request>();
  req->pose_with_covariance = *input_pose_msg;
  req->seq = ++request_id_;

  RCLCPP_INFO(get_logger(), "call NDT Align Server");
  auto result = ndt_client_->async_send_request(req).get();

  if (!result->success) {
    RCLCPP_INFO(get_logger(), "failed NDT Align Server");
    response_id_ = result->seq;
    return false;
  }

  RCLCPP_INFO(get_logger(), "called NDT Align Server");
  response_id_ = result->seq;
  // NOTE temporary cov
  // TODO: output_pose_covariance_
  geometry_msgs::msg::PoseWithCovarianceStamped & pose_with_cov = result->pose_with_covariance;
  pose_with_cov.pose.covariance[0] = 1.0;
  pose_with_cov.pose.covariance[1 * 6 + 1] = 1.0;
  pose_with_cov.pose.covariance[2 * 6 + 2] = 0.01;
  pose_with_cov.pose.covariance[3 * 6 + 3] = 0.01;
  pose_with_cov.pose.covariance[4 * 6 + 4] = 0.01;
  pose_with_cov.pose.covariance[5 * 6 + 5] = 0.2;
  initial_pose_pub_->publish(pose_with_cov);
  enable_gnss_callback_ = false;

  return true;
}
*/
