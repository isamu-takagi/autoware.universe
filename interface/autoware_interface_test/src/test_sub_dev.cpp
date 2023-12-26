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
#include <vehicle_control_interface/steering_status/dev.hpp>

class TestSubDev : public rclcpp::Node
{
public:
  TestSubDev();

private:
  using Interface = vehicle_control_interface::steering_status::dev::Interface;
  autoware_interface_library::Subscription<Interface>::SharedPtr sub_;
  void on_msg(const Interface::Message & msg);
};

TestSubDev::TestSubDev() : Node("test_sub_dev")
{
  autoware_interface_library::AutowareInterfaceAdaptor interface(this);
  sub_ = interface.create_subscription<Interface>(
    std::bind(&TestSubDev::on_msg, this, std::placeholders::_1));
}

void TestSubDev::on_msg(const Interface::Message & msg)
{
  RCLCPP_INFO(get_logger(), "%f", msg.steering_tire_angle);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<TestSubDev>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
