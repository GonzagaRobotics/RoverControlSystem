import rclpy
from rclpy.node import Node, Subscription, Publisher, Timer
from rclpy.clock import Time, Duration

from std_msgs.msg import String
from rcs_interfaces.msg import Heartbeat


SELF_ENTITY_NAME = "rcs_core"
GUI_ENTITY_NAME = "rover_gui"


class Core(Node):
    heartbeat_sub: Subscription
    heartbeat_pub: Publisher

    def __init__(self):
        super().__init__(SELF_ENTITY_NAME)

        self.heartbeat_sub = self.create_subscription(
            Heartbeat,
            f"/{GUI_ENTITY_NAME}/heartbeat/pub",
            self.heartbeat_received,
            10
        )

        self.heartbeat_pub = self.create_publisher(
            Heartbeat,
            f"/{GUI_ENTITY_NAME}/heartbeat/sub",
            10
        )

        self.get_logger().info("RCS Core initialized")

    def heartbeat_received(self, receivedMsg: Heartbeat):
        # Send a heartbeat back
        sentMsg = Heartbeat()
        sentMsg.header.stamp = self.get_clock().now().to_msg()
        sentMsg.entity_name = SELF_ENTITY_NAME

        self.heartbeat_pub.publish(sentMsg)


def main(args=None):
    rclpy.init(args=args)

    core = Core()

    rclpy.spin(core)

    core.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
