// Copyright 2023 The Autoware Contributors
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

#ifndef AUTOWARE_INTERFACE_LIBRARY__IMPL__MESSAGE_HPP_
#define AUTOWARE_INTERFACE_LIBRARY__IMPL__MESSAGE_HPP_

#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

namespace autoware_interface_library
{

template <class T>
using Publisher = rclcpp::Publisher<typename T::Adaptor>;

template <class T>
using Subscription = rclcpp::Subscription<typename T::Adaptor>;

}  // namespace autoware_interface_library

#endif  // AUTOWARE_INTERFACE_LIBRARY__IMPL__MESSAGE_HPP_
