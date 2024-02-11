// Copyright 2024 The Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#include "interface/sample_service/v2.hpp"

#include <component_interface_manager_cpp/cimcpp.hpp>
#include <rclcpp/rclcpp.hpp>

class TestNode : public rclcpp::Node
{
public:
  TestNode();

private:
  using SampleService = interface::sample_service::v2::Service;
  cimcpp::Client<SampleService>::SharedPtr cli_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr group_;
  void on_timer();
};

TestNode::TestNode() : Node("cpp_cli_v2")
{
  cimcpp::ComponentInterfaceManager manager(this);
  group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cli_ = manager.create_client<SampleService>(group_);

  const auto period = rclcpp::Rate(1.0).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });
}

void TestNode::on_timer()
{
  const auto callback = [this](SampleService::Response::SharedPtr res) {
    RCLCPP_INFO_STREAM(get_logger(), "result: " << res->result);
  };

  SampleService::Request::SharedPtr req = std::make_shared<SampleService::Request>();
  req->values = {3, 4};
  RCLCPP_INFO_STREAM(get_logger(), "values: " << req->values[0] << " " << req->values[1]);

  auto future = cli_->async_send_request(req, callback);
  SampleService::Response::SharedPtr res = future.get();
  RCLCPP_INFO_STREAM(get_logger(), "result: " << res->result);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<TestNode>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
