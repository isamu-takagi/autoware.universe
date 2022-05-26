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

#ifndef DEFAULT_AD_API__SPECS__INTERNAL__ROUTE__RESET_HPP_
#define DEFAULT_AD_API__SPECS__INTERNAL__ROUTE__RESET_HPP_

#include <std_srvs/srv/trigger.hpp>

namespace internal_api::route::reset
{

struct T
{
  using Service = std_srvs::srv::Trigger;
  static constexpr char name[] = "/autoware/reset_route";
};

}  // namespace internal_api::route::reset

#endif  // DEFAULT_AD_API__SPECS__INTERNAL__ROUTE__RESET_HPP_
