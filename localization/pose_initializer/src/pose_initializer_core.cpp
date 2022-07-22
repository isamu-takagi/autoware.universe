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

#include "copy_vector_to_array.hpp"

#include <memory>
#include <vector>

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

  // interfaces
  const auto on_initialize = std::bind(&PoseInitializer::OnInitialize, this, _1, _2);
  node.init_pub(pub_state_);
  node.init_srv(srv_initialize_, on_initialize, service_callback_group_);
  pub_align_ = create_publisher<PoseWithCovarianceStamped>("ekf_reset_srv", 1);
  cli_align_ = create_client<RequestPoseAlignment>("ndt_align_srv");

  // parameters
  const auto covariance = declare_parameter<std::vector<double>>("output_pose_covariance");
  CopyVectorToArray(covariance, output_pose_covariance_);

  // other variables
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
  // NOTE: This function is not executed during initialization because mutually exclusive.
  try {
    ChangeState(State::Message::INITIALIZING);
    const auto request_pose = req->pose.empty() ? GetGnssPose() : req->pose.front();
    const auto aligned_pose = AlignPose(request_pose);
    pub_align_->publish(aligned_pose);
    res->status.success = true;
    ChangeState(State::Message::INITIALIZED);
  } catch (const ServiceException & error) {
    res->status = error.status();
    ChangeState(State::Message::UNINITIALIZED);
  }
}

PoseWithCovarianceStamped PoseInitializer::GetGnssPose()
{
  // PoseWithCovarianceStamped pose;
  // return pose;
  // ServiceException(Initialize::Service::Response::ERROR_GNSS, "");
  throw ServiceException(
    Initialize::Service::Response::ERROR_GNSS_SUPPORT, "GNSS is not supported.");
}

PoseWithCovarianceStamped PoseInitializer::AlignPose(const PoseWithCovarianceStamped & pose)
{
  const auto req = std::make_shared<RequestPoseAlignment::Request>();
  req->pose_with_covariance = pose;

  if (!cli_align_->service_is_ready()) {
    throw component_interface_utils::ServiceUnready("NDT align server is not ready.");
  }

  RCLCPP_INFO(get_logger(), "call NDT align server");
  const auto res = cli_align_->async_send_request(req).get();
  RCLCPP_INFO(get_logger(), "called NDT align server");
  if (!res->success) {
    throw ServiceException(
      Initialize::Service::Response::ERROR_ESTIMATION, "NDT align server failed.");
  }

  // Overwrite the covariance.
  res->pose_with_covariance.pose.covariance = output_pose_covariance_;
  return res->pose_with_covariance;
}
