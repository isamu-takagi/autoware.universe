# Copyright 2023 The Autoware Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#         http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from rclpy.node import Node
from tier4_system_msgs.msg import DiagGraphStatus
from tier4_system_msgs.msg import DiagGraphStruct

from .graph import Graph
from .utils import default_qos
from .utils import durable_qos


class MonitorModule:
    def __init__(self, graph: Graph, node: Node):
        self.node = node
        self.sub_struct = None
        self.sub_status = None
        self.graph = graph
        self.graph.append_callback(self.subscribe_status)
        self.subscribe_struct()

    def subscribe_struct(self):
        self.sub_struct = self.node.create_subscription(
            DiagGraphStruct, "/diagnostics_graph/struct", self.graph.create, durable_qos(1)
        )

    def subscribe_status(self):
        self.sub_status = self.node.create_subscription(
            DiagGraphStatus, "/diagnostics_graph/status", self.graph.update, default_qos(1)
        )

    def shutdown(self):
        self.node.destroy_subscription(self.sub_struct)
        self.node.destroy_subscription(self.sub_status)
