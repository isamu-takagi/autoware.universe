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

from python_qt_binding import QtCore
from python_qt_binding import QtWidgets

from .graph import Graph


class MonitorItem:
    def __init__(self, node):
        self.item = QtWidgets.QTreeWidgetItem([node.status.name])
        self.node = node


class MonitorWidget(QtWidgets.QSplitter):
    def __init__(self, graph: Graph):
        super().__init__()
        self.graph = graph
        self.items = {}
        self.root_items = []
        self.tree = QtWidgets.QTreeWidget()
        self.addWidget(self.tree)
        self.addWidget(QtWidgets.QLabel("TEST"))

        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self.on_timer)
        self._timer.start(1000)

    def shutdown(self):
        pass

    def on_timer(self):
        pass

    def dummy(self):
        for node in self.module.graph.nodes:
            p = len(node.parents)
            c = len(node.links)
            print(f"({p}, {c}) {node.status.name}")

            if node.status.name not in self.items:
                self.items[node.status.name] = MonitorItem(node)
            self.items[node.status.name].node = node

        root_items = []
        for item in self.items.values():
            if len(item.node.parents) != 1:
                root_items.append(item)

        if root_items != self.root_items:
            for item in root_items:
                self.tree.addTopLevelItem(item.item)

        print(root_items == self.root_items)
        self.root_items = root_items
