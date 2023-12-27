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
#include <vehicle_control_interface/steering_status/raw.hpp>

class TestSubRaw : public rclcpp::Node
{
public:
  TestSubRaw();

private:
  using Interface = vehicle_control_interface::steering_status::raw::Interface;
  autoware_interface_library::Subscription<Interface>::SharedPtr sub_;
  void on_msg(const Interface::Message::SharedPtr msg);
};

TestSubRaw::TestSubRaw() : Node("test_sub_raw")
{
  autoware_interface_library::AutowareInterfaceAdaptor interface(this);
  sub_ = interface.create_subscription<Interface>(
    std::bind(&TestSubRaw::on_msg, this, std::placeholders::_1));
}

void TestSubRaw::on_msg(const Interface::Message::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "%f", msg->steering_tire_angle);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<TestSubRaw>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
