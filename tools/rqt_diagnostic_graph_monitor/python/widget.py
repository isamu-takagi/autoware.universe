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
    def __init__(self, unit):
        self.item = QtWidgets.QTreeWidgetItem([unit.struct.path])
        self.unit = unit


class MonitorWidget(QtWidgets.QSplitter):
    def __init__(self, graph: Graph):
        super().__init__()
        self.graph = graph
        self.items = []
        self.tree = QtWidgets.QTreeWidget()
        self.addWidget(self.tree)
        # self.addWidget(QtWidgets.QLabel("TEST"))

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

        for unit in self.graph.units:
            p = len(unit.parents)
            c = len(unit.children)
            print(f"({p}, {c}) {unit.struct.path}")

        self.temp_ready = True
        self.items = [MonitorItem(unit) for unit in self.graph.units]

        # branch items
        root_items = []
        for item in self.items:
            if len(item.unit.parents) != 1:
                root_items.append(item)
        for item in root_items:
            self.tree.addTopLevelItem(item.item)
