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
from .utils import QoS


class MonitorModule:
    def __init__(self, graph: Graph, node: Node):
        self.graph = graph
        self.node = node
        self.sub_struct = node.create_subscription(
            DiagGraphStruct, "/diagnostics_graph/struct", graph.create, QoS(1).transient_local()
        )
        self.sub_status = node.create_subscription(
            DiagGraphStatus, "/diagnostics_graph/status", graph.update, QoS(1)
        )

    def shutdown(self):
        self.node.destroy_subscription(self.sub_struct)
        self.node.destroy_subscription(self.sub_status)
