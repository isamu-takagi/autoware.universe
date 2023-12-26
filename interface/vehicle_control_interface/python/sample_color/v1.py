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

import interface_msgs.msg
from rclpy.qos import DurabilityPolicy
from rclpy.qos import ReliabilityPolicy
from vehicle_control_interface.message_interface_base import MessageInterfaceBase


class CustomMessage:
    def __init__(self):
        self.r = 0.0
        self.g = 0.0
        self.b = 0.0


def convert_to_rosidl(custom):
    rosidl = interface_msgs.msg.SampleMessage()
    rosidl.has_alpha = False
    rosidl.color.r = custom.r
    rosidl.color.g = custom.g
    rosidl.color.b = custom.b
    return rosidl


def convert_to_custom(rosidl):
    custom = CustomMessage()
    custom.r = rosidl.color.r
    custom.g = rosidl.color.g
    custom.b = rosidl.color.b
    return custom


Interface = MessageInterfaceBase()
Interface.RosType = interface_msgs.msg.SampleMessage
Interface.Message = CustomMessage
Interface.name = "/sample/color"
Interface.reliability = ReliabilityPolicy.RELIABLE
Interface.durability = DurabilityPolicy.TRANSIENT_LOCAL
Interface.depth = 1
Interface.convert_to_custom = convert_to_custom
Interface.convert_to_rosidl = convert_to_rosidl
