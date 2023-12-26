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


class PyPubV1(rclpy.node.Node):
    def __init__(self):
        super().__init__("py_pub_v1")
        self.manager = ComponentInterfaceManager(self)
        self.pub = self.manager.create_publisher(SampleColor)
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        msg = SampleColor.Message()
        msg.r = 11.0
        msg.g = 12.0
        msg.b = 13.0
        self.pub.publish(msg)


if __name__ == "__main__":
    rclpy.init()
    rclpy.spin(PyPubV1())
    rclpy.shutdown()
