# Copyright 2023 Takagi Isamu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from autoware_interface_library.interface_manager import ComponentInterfaceManager
from autoware_interface_library.sample_color.v1 import Interface as SampleColor
import rclpy
import rclpy.node


class PySubV1(rclpy.node.Node):
    def __init__(self):
        super().__init__("py_sub_v1")
        self.manager = ComponentInterfaceManager(self)
        self.sub = self.manager.create_subscription(SampleColor, self.on_msg)

    def on_msg(self, msg):
        logger = self.get_logger()
        logger.info(f"r: {msg.r}")
        logger.info(f"g: {msg.g}")
        logger.info(f"b: {msg.b}")
        logger.info("---")


if __name__ == "__main__":
    rclpy.init()
    rclpy.spin(PySubV1())
    rclpy.shutdown()