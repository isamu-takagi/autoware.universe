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

#include "pose_initializer_core_old.hpp"

#include "copy_vector_to_array.hpp"

#include <pcl_conversions/pcl_conversions.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

PoseInitializer::PoseInitializer()
: Node("pose_initializer"), tf2_listener_(tf2_buffer_), map_frame_("map")
{
  enable_gnss_callback_ = this->declare_parameter("enable_gnss_callback", true);

  const std::vector<double> initialpose_particle_covariance =
    this->declare_parameter<std::vector<double>>("initialpose_particle_covariance");
  CopyVectorToArray(initialpose_particle_covariance, initialpose_particle_covariance_);

  const std::vector<double> gnss_particle_covariance =
    this->declare_parameter<std::vector<double>>("gnss_particle_covariance");
  CopyVectorToArray(gnss_particle_covariance, gnss_particle_covariance_);

  const std::vector<double> service_particle_covariance =
    this->declare_parameter<std::vector<double>>("service_particle_covariance");
  CopyVectorToArray(service_particle_covariance, service_particle_covariance_);

  const std::vector<double> output_pose_covariance =
    this->declare_parameter<std::vector<double>>("output_pose_covariance");
  CopyVectorToArray(output_pose_covariance, output_pose_covariance_);

  // We can't use _1 because pcl leaks an alias to boost::placeholders::_1, so it would be ambiguous

  gnss_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "gnss_pose_cov", 1,
    std::bind(&PoseInitializer::callbackGNSSPoseCov, this, std::placeholders::_1));
}

void PoseInitializer::serviceInitializePose(
  const tier4_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
  tier4_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res)
{
  enable_gnss_callback_ = false;  // get only first topic

  auto add_height_pose_msg_ptr = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  getHeight(req->pose_with_covariance, add_height_pose_msg_ptr);

  add_height_pose_msg_ptr->pose.covariance = service_particle_covariance_;

  res->success = callAlignServiceAndPublishResult(add_height_pose_msg_ptr);
}

void PoseInitializer::callbackInitialPose(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_cov_msg_ptr)
{
  enable_gnss_callback_ = false;  // get only first topic

  auto add_height_pose_msg_ptr = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  getHeight(*pose_cov_msg_ptr, add_height_pose_msg_ptr);

  add_height_pose_msg_ptr->pose.covariance = initialpose_particle_covariance_;

  callAlignServiceAndPublishResult(add_height_pose_msg_ptr);
}

// NOTE Still not usable callback
void PoseInitializer::callbackGNSSPoseCov(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_cov_msg_ptr)
{
  if (!enable_gnss_callback_) {
    return;
  }

  // TODO(YamatoAndo) check service is available

  auto add_height_pose_msg_ptr = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  getHeight(*pose_cov_msg_ptr, add_height_pose_msg_ptr);

  add_height_pose_msg_ptr->pose.covariance = gnss_particle_covariance_;

  callAlignServiceAndPublishResult(add_height_pose_msg_ptr);
}
