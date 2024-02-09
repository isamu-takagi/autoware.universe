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

class TestCliDev : public rclcpp::Node
{
public:
  TestCliDev();

private:
  using Interface = vehicle_control_interface::control_mode_request::dev::Interface;
  autoware_interface_library::Client<Interface>::SharedPtr cli_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr group_;
  void on_timer();
};

TestCliDev::TestCliDev() : Node("test_cli_dev")
{
  autoware_interface_library::ComponentInterfaceManager manager(this);
  group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cli_ = manager.create_client<Interface>(group_);

  const auto period = rclcpp::Rate(1.0).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });
}

void TestCliDev::on_timer()
{
  const auto callback = [this](Interface::Response::SharedPtr res) {
    RCLCPP_INFO_STREAM(get_logger(), "res1:" << std::boolalpha << res->success2);
  };

  Interface::Request::SharedPtr req = std::make_shared<Interface::Request>();
  req->mode2 = Interface::Service::Mode::AUTONOMOUS;
  RCLCPP_INFO_STREAM(get_logger(), "req");

  auto future = cli_->async_send_request(req, callback);
  Interface::Response::SharedPtr res = future.get();
  RCLCPP_INFO_STREAM(get_logger(), "res2:" << std::boolalpha << res->success2);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<TestCliDev>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
