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

from .utils import foreach


class BaseUnit:
    def __init__(self):
        self._parents = []
        self._children = []
        self._path = None

    @property
    def parents(self):
        return self._parents

    @property
    def children(self):
        return self._children

    @property
    def path(self):
        return self._path


class NodeUnit(BaseUnit):
    def __init__(self, struct):
        super().__init__()
        self._path = struct.path

    def update(self, status):
        self._status = status


class DiagUnit(BaseUnit):
    def __init__(self, struct):
        super().__init__()
        self._path = struct.path
        self._name = struct.name

    def update(self, status):
        self._status = status


class UnitLink:
    def __init__(self, child: BaseUnit):
        self._parent = None
        self._child = child

    def update(self, status):
        self.status = status

    @property
    def parent(self):
        return self._parent

    @property
    def child(self):
        return self._child


class Graph:
    def __init__(self):
        self._callbacks = []
        self._nodes = []
        self._diags = []
        self._units = []
        self._links = []

    def create(self, msg):
        # Create graph units and links.
        self._nodes = [NodeUnit(struct) for struct in msg.nodes]
        self._diags = [DiagUnit(struct) for struct in msg.diags]
        self._units = self._nodes + self._diags
        for struct in msg.links:
            units = self._diags if struct.is_leaf else self._nodes
            child = units[struct.child]
            self._links.append(UnitLink(child))

        # Update references.
        for node, struct in zip(self._nodes, msg.nodes):
            node._children = [self._links[index] for index in struct.links]
            for link in node._children:
                link._parent = node
        for link in self._links:
            unit = link.child
            unit._parents.append(link)

        # Notify graph created.
        foreach(self._callbacks, lambda callback: callback())

    def update(self, msg):
        for node, status in zip(self._nodes, msg.nodes):
            node.update(status)
        for diag, status in zip(self._diags, msg.diags):
            diag.update(status)
        for link, status in zip(self._links, msg.links):
            link.update(status)

    def append_callback(self, callback):
        self._callbacks.append(callback)

    @property
    def units(self):
        return self._units

    @property
    def links(self):
        return self._links
