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

#include <component_interface_utils/response.hpp>

#include <memory>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

PoseInitializer::PoseInitializer() : Node("pose_initializer")
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  const auto node = component_interface_utils::NodeAdaptor(this);
  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  const auto on_initialize = std::bind(&PoseInitializer::OnInitialize, this, _1, _2);
  node.init_pub(pub_state_);
  node.init_srv(srv_initialize_, on_initialize, service_callback_group_);

  pub_align_ = create_publisher<PoseWithCovarianceStamped>("pub_align", 1);
  cli_align_ = create_client<RequestPoseAlignment>("srv_align");
  // while (!cli_align_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
  //   RCLCPP_INFO(get_logger(), "Waiting for service...");
  // }

  ChangeState(State::Message::UNINITIALIZED);
}

void PoseInitializer::ChangeState(State::Message::_state_type state)
{
  state_.stamp = now();
  state_.state = state;
  pub_state_->publish(state_);
}

void PoseInitializer::OnInitialize(ROS_SERVICE_ARG(Initialize, req, res))
{
  if (state_.state == State::Message::INITIALIZING) {
    throw component_interface_utils::ServiceException(
      "message", 0);  // TODO(Takagi, Isamu): error code
  }

  ChangeState(State::Message::INITIALIZING);
  const auto request_pose = req->pose.empty() ? GetGnssPose() : req->pose.front();
  const auto aligned_pose = AlignPose(request_pose);
  // set output pose cov
  pub_align_->publish(aligned_pose);
  ChangeState(State::Message::INITIALIZED);

  res->status = component_interface_utils::response_success();
}

PoseWithCovarianceStamped PoseInitializer::GetGnssPose()
{
  throw component_interface_utils::ServiceException(
    "No GNSS", 0);  // TODO(Takagi, Isamu): error code
  // PoseWithCovarianceStamped pose;
  // return pose;
}

PoseWithCovarianceStamped PoseInitializer::AlignPose(const PoseWithCovarianceStamped & pose)
{
  const auto req = std::make_shared<RequestPoseAlignment::Request>();
  req->seq = 0;
  req->pose_with_covariance = pose;

  RCLCPP_INFO(get_logger(), "call NDT Align Server");
  const auto res = cli_align_->async_send_request(req).get();
  RCLCPP_INFO(get_logger(), "called NDT Align Server");

  if (!res->success) {
    RCLCPP_INFO(get_logger(), "failed NDT Align Server");
    throw component_interface_utils::ServiceException(
      "NDT alignment failed", 0);  // TODO(Takagi, Isamu): error code
  }

  // Overwrite the covariance.
  res->pose_with_covariance.pose.covariance = std::array<double, 36>();  // output_pose_covariance_
  return res->pose_with_covariance;
}
