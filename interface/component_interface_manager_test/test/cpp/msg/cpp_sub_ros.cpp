// Copyright 2024 The Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#include "interface/sample_message/ros.hpp"

#include <component_interface_manager_cpp/cimcpp.hpp>
#include <rclcpp/rclcpp.hpp>

class TestNode : public rclcpp::Node
{
public:
  TestNode();

private:
  using SampleMessage = interface::sample_message::ros::Message;
  cimcpp::Subscription<SampleMessage>::SharedPtr sub_;
  void on_msg(const SampleMessage & msg);
};

TestNode::TestNode() : Node("cpp_sub_ros")
{
  cimcpp::ComponentInterfaceManager manager(this);
  sub_ = manager.create_subscription<SampleMessage>(
    std::bind(&TestNode::on_msg, this, std::placeholders::_1));
}

void TestNode::on_msg(const SampleMessage & msg)
{
  RCLCPP_INFO(get_logger(), "%.2f %.2f", msg.linear.x, msg.angular.z);
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
