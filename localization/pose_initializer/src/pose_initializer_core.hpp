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

#ifndef POSE_INITIALIZER_CORE_HPP_
#define POSE_INITIALIZER_CORE_HPP_

#include <component_interface_specs/localization/initialization.hpp>
#include <component_interface_utils/macros.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tier4_localization_msgs/srv/pose_with_covariance_stamped.hpp>

using ServiceException = component_interface_utils::ServiceException;
using Initialize = localization_interface::initialization::Initialize;
using State = localization_interface::initialization::State;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using RequestPoseAlignment = tier4_localization_msgs::srv::PoseWithCovarianceStamped;

class PoseInitializer : public rclcpp::Node
{
public:
  PoseInitializer();

private:
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_align_;
  rclcpp::Client<RequestPoseAlignment>::SharedPtr cli_align_;
  component_interface_utils::Publisher<State>::SharedPtr pub_state_;
  component_interface_utils::Service<Initialize>::SharedPtr srv_initialize_;
  State::Message state_;
  std::array<double, 36> output_pose_covariance_;

  void ChangeState(State::Message::_state_type state);
  void OnInitialize(API_SERVICE_ARG(Initialize, res, req));
  PoseWithCovarianceStamped GetGnssPose();
  PoseWithCovarianceStamped AlignPose(const PoseWithCovarianceStamped & pose);
};

#endif  // POSE_INITIALIZER_CORE_HPP_
