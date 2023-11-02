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
        self.parents = []

    def update(self, nodes):
        self.links = [(nodes[link.index], link.used) for link in self.links]
        for link, used in self.links:
            link.parents.append((self, used))


class Graph:
    def __init__(self, graph):
        self.nodes = [Node(node) for node in graph.nodes]
        for node in self.nodes:
            node.update(self.nodes)
