# Copyright 2024 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0


class ComponentInterfaceManager:
    def __init__(self, node):
        self.node = node

    def create_subscription(self, interface, callback):
        profile = interface._get_profile()
        adaptor = interface._get_adaptor()

        def wrapped(msg):
            callback(adaptor.convert_to_custom(msg))

        name = profile.get_name()
        qos = profile.get_sub_qos()
        msg = adaptor.get_ros_type()
        return self.node.create_subscription(msg, name, wrapped, qos)

    def create_publisher(self, interface):
        profile = interface._get_profile()
        adaptor = interface._get_adaptor()
        name = profile.get_name()
        qos = profile.get_pub_qos()
        msg = adaptor.get_ros_type()
        pub = self.node.create_publisher(msg, name, qos)
        return WrappedPublisher(pub, adaptor.convert_to_ros_message)


class WrappedPublisher:
    def __init__(self, pub, convert):
        self.pub = pub
        self.convert = convert

    def publish(self, msg):
        self.pub.publish(self.convert(msg))
