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

Interface = MessageInterfaceBase()
Interface.RosType = interface_msgs.msg.SampleMessage
Interface.Message = interface_msgs.msg.SampleMessage
Interface.name = "/sample/color"
Interface.reliability = ReliabilityPolicy.RELIABLE
Interface.durability = DurabilityPolicy.TRANSIENT_LOCAL
Interface.depth = 1
