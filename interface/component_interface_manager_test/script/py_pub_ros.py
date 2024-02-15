# Copyright 2024 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

from component_interface_manager_py import ComponentInterfaceManager
from component_interface_manager_test.sample_message.ros import Message as SampleMessage
import rclpy
import rclpy.node


class TestNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("py_pub_ros")
        self.manager = ComponentInterfaceManager(self)
        self.pub = self.manager.create_publisher(SampleMessage)
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        msg = SampleMessage()
        msg.velocity = 300.00
        msg.steering = 333.33
        self.pub.publish(msg)


if __name__ == "__main__":
    try:
        rclpy.init()
        rclpy.spin(TestNode())
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
