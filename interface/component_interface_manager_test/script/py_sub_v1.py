# Copyright 2024 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

from component_interface_manager_py import ComponentInterfaceManager
from component_interface_manager_test.sample_message.v1 import Message as SampleMessage
import rclpy
import rclpy.node


class TestNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("py_sub_v1")
        self.manager = ComponentInterfaceManager(self)
        self.sub = self.manager.create_subscription(SampleMessage, self.on_msg)

    def on_msg(self, msg):
        logger = self.get_logger()
        logger.info(f"velocity: {msg.velocity:.2f}")
        logger.info(f"steering: {msg.steering:.2f}")
        logger.info("---")


if __name__ == "__main__":
    try:
        rclpy.init()
        rclpy.spin(TestNode())
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
