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

class TestPubRaw : public rclcpp::Node
{
public:
  TestPubRaw();

private:
  using Interface = vehicle_control_interface::steering_status::raw::Interface;
  autoware_interface_library::Publisher<Interface>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  void on_timer();
};

TestPubRaw::TestPubRaw() : Node("test_pub_raw")
{
  autoware_interface_library::AutowareInterfaceAdaptor interface(this);
  pub_ = interface.create_publisher<Interface>();

  const auto period = rclcpp::Rate(1.0).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });
}

void TestPubRaw::on_timer()
{
  Interface::Message msg;
  pub_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<TestPubRaw>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
