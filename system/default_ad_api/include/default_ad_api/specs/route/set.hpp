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

#ifndef DEFAULT_AD_API__SPECS__ROUTE__SET_HPP_
#define DEFAULT_AD_API__SPECS__ROUTE__SET_HPP_

#include <autoware_ad_api_msgs/srv/route_set.hpp>

namespace ad_api::route::set
{

struct T
{
  using Service = autoware_ad_api_msgs::srv::RouteSet;
  static constexpr char name[] = "/api/route/set";
};

}  // namespace ad_api::route::set

#endif  // DEFAULT_AD_API__SPECS__ROUTE__SET_HPP_
