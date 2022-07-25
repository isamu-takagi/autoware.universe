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

#ifndef INITIALPOSE_RVIZ_HELPER_HPP_
#define INITIALPOSE_RVIZ_HELPER_HPP_

#include "fitting_to_map_height.hpp"

#include <component_interface_specs/localization/initialization.hpp>
#include <component_interface_utils/macros.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

using Initialize = localization_interface::initialization::Initialize;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

class InitialPoseRvizHelper : public rclcpp::Node
{
public:
  InitialPoseRvizHelper();

private:
  FittingMapHeight fit_map_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
  component_interface_utils::Client<Initialize>::SharedPtr cli_initialize_;
  std::array<double, 36> rviz_particle_covariance_;

  void OnInitialPose(PoseWithCovarianceStamped::ConstSharedPtr msg);
};

#endif  // INITIALPOSE_RVIZ_HELPER_HPP_
