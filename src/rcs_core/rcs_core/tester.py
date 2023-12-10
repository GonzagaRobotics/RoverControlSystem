import rclpy
import time
from rclpy.node import Node, Subscription, Timer
from rclpy.clock import Time, Duration

from rcs_interfaces.msg import Heartbeat


ENTITY_NAME = "tester"

HEARTBEAT_TIMER_INTERVAL = 5.0
"""About how long to wait between expected heartbeats."""


class Tester(Node):
    def __init__(self):
        super().__init__(ENTITY_NAME)

        self.heartbeat_pub = self.create_publisher(
            Heartbeat, "rcs_core/heartbeat/sub", 10)

        time.sleep(1)

        self.send_heartbeat()

        self.heartbeat_timer = self.create_timer(
            HEARTBEAT_TIMER_INTERVAL, self.send_heartbeat)

        self.get_logger().info("Tester initialized")

    def send_heartbeat(self):
        msg = Heartbeat()
        msg.node_name = ENTITY_NAME
        msg.header.stamp = self.get_clock().now().to_msg()
        self.get_logger().info("Sending heartbeat")
        self.heartbeat_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    tester = Tester()

    rclpy.spin(tester)

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
