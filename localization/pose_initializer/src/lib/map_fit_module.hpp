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

#ifndef LIB__MAP_FIT_MODULE_HPP_
#define LIB__MAP_FIT_MODULE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>

#include <string>

using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

class MapFitModule
{
public:
  explicit MapFitModule(rclcpp::Node * node);
  PoseWithCovarianceStamped FitHeight(const PoseWithCovarianceStamped pose) const;

private:
  rclcpp::Logger logger_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_map_;
  tf2::BufferCore tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  std::string map_frame_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;

  void OnMap(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  double GetGroundHeight(const tf2::Vector3 & point) const;
};

#endif  // LIB__MAP_FIT_MODULE_HPP_
