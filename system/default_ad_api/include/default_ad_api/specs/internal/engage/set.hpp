// Copyright 2022 TIER IV, Inc.
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

#ifndef DEFAULT_AD_API__SPECS__INTERNAL__ENGAGE__SET_HPP_
#define DEFAULT_AD_API__SPECS__INTERNAL__ENGAGE__SET_HPP_

#include <tier4_external_api_msgs/srv/engage.hpp>

namespace internal_api::engage::set
{

struct T
{
  using Service = tier4_external_api_msgs::srv::Engage;
  static constexpr char name[] = "/api/autoware/set/engage";
};

}  // namespace internal_api::engage::set

#endif  // DEFAULT_AD_API__SPECS__INTERNAL__ENGAGE__SET_HPP_
