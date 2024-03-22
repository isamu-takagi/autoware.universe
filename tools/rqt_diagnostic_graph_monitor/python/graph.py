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

from threading import Lock

from tier4_system_msgs.msg import DiagGraphStatus
from tier4_system_msgs.msg import DiagGraphStruct
from tier4_system_msgs.msg import DiagLeafStatus
from tier4_system_msgs.msg import DiagLeafStruct
from tier4_system_msgs.msg import DiagLinkStatus
from tier4_system_msgs.msg import DiagLinkStruct
from tier4_system_msgs.msg import DiagNodeStatus
from tier4_system_msgs.msg import DiagNodeStruct


class NodeUnit:
    def __init__(self, struct: DiagNodeStruct):
        self.struct = struct
        self.status = None

    def update(self, status: DiagNodeStatus):
        self.status = status


class DiagUnit:
    def __init__(self, struct: DiagLeafStruct):
        self.struct = struct
        self.status = None

    def update(self, status: DiagLeafStatus):
        self.status = status


class UnitLink:
    def __init__(self, struct: DiagLinkStruct):
        self.struct = struct
        self.status = None

    def update(self, status: DiagLinkStatus):
        self.status = status


class Graph:
    def __init__(self):
        self.ready = False
        self.mutex = Lock()
        self.nodes = []
        self.diags = []
        self.links = []

    def create(self, msg: DiagGraphStruct):
        self.nodes = [NodeUnit(struct) for struct in msg.nodes]
        self.diags = [DiagUnit(struct) for struct in msg.diags]
        self.links = [DiagUnit(struct) for struct in msg.links]

    def update(self, msg: DiagGraphStatus):
        for node, status in zip(self.nodes, msg.nodes):
            node.update(status)
        for diag, status in zip(self.diags, msg.diags):
            diag.update(status)
        for link, status in zip(self.links, msg.links):
            link.update(status)