# Copyright 2024 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile


class Message(Twist):
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
        return custom

    @staticmethod
    def convert_to_custom(rosmsg):
        return rosmsg
