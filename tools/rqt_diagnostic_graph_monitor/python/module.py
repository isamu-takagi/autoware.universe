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
from rqt_diagnostic_graph_monitor.graph import Graph
from tier4_system_msgs.msg import DiagnosticGraph


class MonitorModule:
    def __init__(self, node: Node):
        self.node = node
        self.sub = node.create_subscription(DiagnosticGraph, "/diagnostics_graph", self.callback, 5)
        self.graph = None

    def shutdown(self):
        self.node.destroy_subscription(self.sub)

    def callback(self, msg):
        self.graph = Graph(msg)
