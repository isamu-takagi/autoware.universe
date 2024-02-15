# Copyright 2024 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile


@dataclass(slots=True)
class Message:
    velocity: float = 0.0
    steering: float = 0.0

    @staticmethod
    def _get_profile():
        return Profile

    @staticmethod
    def _get_adaptor():
        return Adaptor


class Profile:
    @staticmethod
    def get_name():
        return "/test/message"

    @staticmethod
    def get_pub_qos():
        return QoSProfile(depth=1)

    @staticmethod
    def get_sub_qos():
        return QoSProfile(depth=1)


class Adaptor:
    @staticmethod
    def get_ros_type():
        return Twist

    @staticmethod
    def convert_to_ros_message(custom):
        rosmsg = Twist()
        rosmsg.linear.x = custom.velocity
        rosmsg.angular.z = custom.steering
        return rosmsg

    @staticmethod
    def convert_to_custom(rosmsg):
        custom = Message()
        custom.velocity = rosmsg.linear.x
        custom.steering = rosmsg.angular.z
        return custom
