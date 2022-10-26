// Copyright 2022 Takagi, Isamu
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

#ifndef COMMON__RCLCPPEX__EXAMPLE__LISTENER_HPP_
#define COMMON__RCLCPPEX__EXAMPLE__LISTENER_HPP_

#include <rclcppex/rclcppex.hpp>

#include <std_msgs/msg/string.hpp>

class Listener : public rclcppex::Node
{
public:
  Listener();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcppex::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

#endif  // COMMON__RCLCPPEX__EXAMPLE__LISTENER_HPP_
