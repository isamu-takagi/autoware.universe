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
from .items import MonitorItem


class MonitorWidget(QtWidgets.QSplitter):
    def __init__(self, graph: Graph):
        super().__init__()
        self.graph = graph
        self.items = []
        self.root_tree = QtWidgets.QTreeWidget()
        self.node_tree = QtWidgets.QTreeWidget()
        self.root_tree.setHeaderLabels(["Top-level nodes"])
        self.node_tree.setHeaderLabels(["Intermediate nodes"])
        self.addWidget(self.root_tree)
        self.addWidget(self.node_tree)

        self.temp_ready = False
        self.root_items = []

        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self.on_timer)
        self._timer.start(1000)

    def shutdown(self):
        pass

    def on_timer(self):
        if self.temp_ready:
            return
        if len(self.graph.units) == 0:
            return
        self.temp_ready = True
        # add all link
        # add not 1

        item = MonitorItem(None)
        self.node_tree.addTopLevelItem(item.item)

        """
        for link in self.graph.links:
            MonitorItem(link)

        # branch items
        for item in self.items:
            if len(item.unit.parents) == 0:
                self.root_tree.addTopLevelItem(item.item)
            if len(item.unit.parents) >= 2:
                self.node_tree.addTopLevelItem(item.item)
        """
