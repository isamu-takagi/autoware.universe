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

#include "mock.hpp"

MockNode::MockNode() : Node("mock_node")
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  const auto on_ndt_align = std::bind(&MockNode::OnNdtAlign, this, _1, _2);
  srv_align_ = this->create_service<RequestPoseAlignment>(
    "/localization/pose_estimator/ndt_align_srv", on_ndt_align);
}

void MockNode::OnNdtAlign(ROS_SERVICE_ARG(RequestPoseAlignment, req, res))
{
  res->success = true;
  res->pose_with_covariance = req->pose_with_covariance;
}

int main() {}
