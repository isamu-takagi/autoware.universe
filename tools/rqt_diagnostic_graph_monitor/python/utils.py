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

from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile


class QoS(QoSProfile):
    def __init__(self, depth):
        super().__init__(depth=depth)

    def transient_local(self):
        self.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        return self
