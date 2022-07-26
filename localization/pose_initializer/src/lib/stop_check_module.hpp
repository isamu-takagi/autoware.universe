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

#ifndef LIB__STOP_CHECK_MODULE_HPP_
#define LIB__STOP_CHECK_MODULE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

class StopCheckModule
{
public:
  explicit StopCheckModule(rclcpp::Node * node);
  bool IsStopped() const;

private:
  using TwistWithCovarianceStamped = geometry_msgs::msg::TwistWithCovarianceStamped;
  // rclcpp::Clock::SharedPtr clock_;
  rclcpp::Subscription<TwistWithCovarianceStamped>::SharedPtr sub_twist_;
  TwistWithCovarianceStamped::ConstSharedPtr twist_;
  double duration_;

  void OnTwist(TwistWithCovarianceStamped::ConstSharedPtr msg);
};

#endif  // LIB__STOP_CHECK_MODULE_HPP_
