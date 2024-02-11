// Copyright 2024 The Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#include "interface/sample_service/v1.hpp"

#include <component_interface_manager_cpp/cimcpp.hpp>
#include <rclcpp/rclcpp.hpp>

class TestNode : public rclcpp::Node
{
public:
  TestNode();

private:
  using SampleService = interface::sample_service::v1::Service;
  cimcpp::Service<SampleService>::SharedPtr srv_;
  void on_service(SampleService::Request::SharedPtr req, SampleService::Response::SharedPtr res);
};

TestNode::TestNode() : Node("cpp_srv_v1")
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  cimcpp::ComponentInterfaceManager manager(this);
  srv_ = manager.create_service<SampleService>(std::bind(&TestNode::on_service, this, _1, _2));
}

void TestNode::on_service(
  SampleService::Request::SharedPtr req, SampleService::Response::SharedPtr res)
{
  res->s = req->x + req->y;
  RCLCPP_INFO_STREAM(get_logger(), req->x << " + " << req->y << " = " << res->s);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<TestNode>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
