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

#include <autoware_interface_library/autoware_interface_library.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_control_interface/control_mode_request/dev.hpp>

class TestSrvDev : public rclcpp::Node
{
public:
  TestSrvDev();

private:
  using Interface = vehicle_control_interface::control_mode_request::dev::Interface;
  autoware_interface_library::Service<Interface>::SharedPtr srv_;
  void on_service(Interface::Request::SharedPtr req, Interface::Response::SharedPtr res);
};

TestSrvDev::TestSrvDev() : Node("test_srv_dev")
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  autoware_interface_library::ComponentInterfaceManager manager(this);
  srv_ = manager.create_service<Interface>(std::bind(&TestSrvDev::on_service, this, _1, _2));
}

void TestSrvDev::on_service(Interface::Request::SharedPtr req, Interface::Response::SharedPtr res)
{
  (void)req;
  (void)res;
  RCLCPP_INFO_STREAM(get_logger(), "call service");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<TestSrvDev>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
