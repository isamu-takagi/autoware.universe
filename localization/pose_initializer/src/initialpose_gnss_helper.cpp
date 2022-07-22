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

#include "initialpose_gnss_helper.hpp"

#include <component_interface_specs/localization/initialization.hpp>
#include <component_interface_utils/rclcpp/exceptions.hpp>
using Initialize = localization_interface::initialization::Initialize;

InitialPoseGnssHelper::InitialPoseGnssHelper(rclcpp::Node * node) : fit_map_(node)
{
  sub_gnss_pose_ = node->create_subscription<PoseWithCovarianceStamped>(
    "gnss_pose_cov", 1, std::bind(&InitialPoseGnssHelper::OnGnssPose, this, std::placeholders::_1));
}

void InitialPoseGnssHelper::OnGnssPose(PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  gnss_pose_ = msg;

  const auto & p = msg->pose.pose.position;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("gnss_node"), p.x << " " << p.y << " " << p.z);
}

PoseWithCovarianceStamped InitialPoseGnssHelper::GetPose() const
{
  if (!gnss_pose_) {
    throw component_interface_utils::ServiceException(
      Initialize::Service::Response::ERROR_GNSS, "A GNSS pose has not been received.");
  }
  // TODO(Takagi, Isamu): check time stamp
  return *gnss_pose_;
}
