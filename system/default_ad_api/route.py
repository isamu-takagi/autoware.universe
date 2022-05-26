from autoware_ad_api_msgs.srv import RouteSet
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node


class RouteTestNode(Node):
    def __init__(self):
        super().__init__("route_test_node")
        self.cli = self.create_client(RouteSet, "/api/route/set")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.future = self.send_request()

    def send_request(self):
        request = RouteSet.Request()
        request.route.header.frame_id = "map"
        request.route.header.stamp = self.get_clock().now().to_msg()
        request.route.start = [self.start()]
        request.route.goal = self.goal()
        request.route.waypoints = [self.wp1(), self.wp2()]
        return self.cli.call_async(request)

    @staticmethod
    def start():
        pose = Pose()
        pose.position.x = 3702.835693359375
        pose.position.y = 73765.6640625
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.22806857443146358
        pose.orientation.w = 0.9736450715516409
        return pose

    @staticmethod
    def goal():
        pose = Pose()
        pose.position.x = 3814.58935546875
        pose.position.y = 73773.15625
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.23543656297405313
        pose.orientation.w = 0.9718897184428719
        return pose

    @staticmethod
    def wp1():
        pose = Pose()
        pose.position.x = 3818.707275390625
        pose.position.y = 73798.8125
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = -0.5272406365743172
        pose.orientation.w = 0.8497160179405286
        return pose

    @staticmethod
    def wp2():
        pose = Pose()
        pose.position.x = 3756.102294921875
        pose.position.y = 73689.7421875
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = -0.9720202475380905
        pose.orientation.w = 0.234897080390518
        return pose


def spin(node):
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            response = node.future.result()
            print(response)
            break


def main(args=None):
    rclpy.init()
    node = RouteTestNode()
    spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
