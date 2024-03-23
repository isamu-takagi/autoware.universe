# Copyright 2024 The Autoware Contributors
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


from python_qt_binding import QtGui
from python_qt_binding import QtWidgets

from .graph import UnitLink


class MonitorIcons:
    def __init__(self, unit):
        self.disable = QtGui.QIcon.fromTheme("dialog-question")
        self.unknown = QtGui.QIcon.fromTheme("system-search")
        self.icon = QtGui.QIcon.fromTheme("emblem-default")
        self.warn = QtGui.QIcon.fromTheme("emblem-important")  # or dialog-warning
        self.error = QtGui.QIcon.fromTheme("dialog-error")
        self.stale = QtGui.QIcon.fromTheme("appointment-missed")


class MonitorItem:
    def __init__(self, link: UnitLink):
        self.item = QtWidgets.QTreeWidgetItem(["test"])
        self.link = link
