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

#ifndef INITIALPOSE_GNSS_HELPER_HPP_
#define INITIALPOSE_GNSS_HELPER_HPP_

#include "fitting_to_map_height.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

class InitialPoseGnssHelper
{
public:
  explicit InitialPoseGnssHelper(rclcpp::Node * node);
  PoseWithCovarianceStamped GetPose() const;

private:
  FittingMapHeight fit_map_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_gnss_pose_;
  PoseWithCovarianceStamped::ConstSharedPtr gnss_pose_;

  void OnGnssPose(PoseWithCovarianceStamped::ConstSharedPtr msg);
};

#endif  // INITIALPOSE_GNSS_HELPER_HPP_
