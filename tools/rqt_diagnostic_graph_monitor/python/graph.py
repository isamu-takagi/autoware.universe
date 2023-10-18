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


class Node:
    def __init__(self, node):
        self.status = node.status
        self.links = node.links
        self.parents = 0


class Graph:
    def __init__(self, graph):
        self.nodes = [Node(node) for node in graph.nodes]
        for node in self.nodes:
            for link in node.links:
                self.nodes[link.index].parents += 1

    def dump(self, logger):
        for node in self.nodes:
            t = self.node_type(len(node.links), node.parents)
            logger.info(f"{t}: {node.status.name}")
        logger.info("==========")

    @staticmethod
    def node_type(c, p):
        if p == 0:
            return "R"
        if c == 0:
            return "L"
        return "U"
