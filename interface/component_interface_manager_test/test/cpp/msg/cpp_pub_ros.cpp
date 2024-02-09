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
  cimcpp::Publisher<SampleMessage>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  void on_timer();
};

TestNode::TestNode() : Node("cpp_pub_ros")
{
  cimcpp::ComponentInterfaceManager manager(this);
  pub_ = manager.create_publisher<SampleMessage>();

  const auto period = rclcpp::Rate(1.0).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });
}

void TestNode::on_timer()
{
  SampleMessage msg;
  msg.linear.x = 30.00;
  msg.angular.z = 33.33;
  pub_->publish(msg);
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
