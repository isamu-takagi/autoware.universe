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

#ifndef COMPONENT_INTERFACE_UTILS__RESPONSE_HPP_
#define COMPONENT_INTERFACE_UTILS__RESPONSE_HPP_

#include <autoware_ad_api_msgs/msg/response_status.hpp>
#include <autoware_ad_api_msgs/msg/response_status_detail.hpp>

namespace component_interface_utils::response
{

using ResponseStatusDetail = autoware_ad_api_msgs::msg::ResponseStatusDetail;

inline ResponseStatusDetail success()
{
  ResponseStatusDetail status;
  status.code = ResponseStatusDetail::SUCCESS;
  return status;
}

}  // namespace component_interface_utils::response

#endif  // COMPONENT_INTERFACE_UTILS__RESPONSE_HPP_
