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

from rclpy.qos import QoSProfile


class WrappedPublisher:
    def __init__(self, pub, convert):
        self.pub = pub
        self.convert = convert

    def publish(self, msg):
        self.pub.publish(self.convert(msg))


class ComponentInterfaceManager:
    def __init__(self, node):
        self.node = node

    def create_subscription(self, interface, callback):
        def wrapped(msg):
            callback(interface.convert_to_custom(msg))

        qos = self.get_qos(interface)
        return self.node.create_subscription(interface.RosType, interface.name, wrapped, qos)

    def create_publisher(self, interface):
        qos = self.get_qos(interface)
        pub = self.node.create_publisher(interface.RosType, interface.name, qos)
        return WrappedPublisher(pub, interface.convert_to_rosidl)

    @staticmethod
    def get_qos(interface):
        qos = QoSProfile(
            depth=interface.depth,
            reliability=interface.reliability,
            durability=interface.durability,
        )
        return qos
