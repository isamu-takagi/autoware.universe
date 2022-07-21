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

class PoseInitializer : public rclcpp::Node
{
public:
  PoseInitializer();

private:
  using Initialize = localization_interface::initialization::Initialize;
  using State = localization_interface::initialization::State;
  component_interface_utils::Publisher<State>::SharedPtr pub_state_;
  component_interface_utils::Service<Initialize>::SharedPtr srv_initialize_;

  void OnInitialize(ROS_SERVICE_ARG(Initialize, res, req));
};

#endif  // POSE_INITIALIZER_CORE_HPP_
