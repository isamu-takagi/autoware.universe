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

#include "test.hpp"

namespace default_ad_api
{

TestNode::TestNode(const rclcpp::NodeOptions & options) : Node("test", options)
{
  const auto node = component_interface_utils::NodeAdaptor(this);
  group_srv_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  node.relay_service(cli_, srv_, group_srv_);
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::TestNode)
