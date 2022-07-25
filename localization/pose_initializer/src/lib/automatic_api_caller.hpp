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

#ifndef LIB__AUTOMATIC_API_CALLER_HPP_
#define LIB__AUTOMATIC_API_CALLER_HPP_

#include <component_interface_specs/localization.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

class AutomaticApiCaller : public rclcpp::Node
{
public:
  AutomaticApiCaller();

private:
  void OnTimer();
  using Initialize = localization_interface::Initialize;
  using State = localization_interface::InitializationState;
  rclcpp::TimerBase::SharedPtr timer_;
  component_interface_utils::Subscription<State>::SharedPtr sub_state_;
  component_interface_utils::Client<Initialize>::SharedPtr cli_initialize_;
  State::Message state_;
};

#endif  // LIB__AUTOMATIC_API_CALLER_HPP_
