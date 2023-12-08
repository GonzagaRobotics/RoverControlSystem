import rclpy
from rclpy.node import Node, Subscription, Timer
from rclpy.clock import Time, Duration

from rcs_interfaces.msg import Heartbeat


SELF_ENTITY_NAME = "rcs_core"
GUI_ENTITY_NAME = "rover_gui"


class Core(Node):
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

    def heartbeat_received(self, msg: Heartbeat):
        time_sent = Time(seconds=msg.header.stamp.sec,
                         nanoseconds=msg.header.stamp.nanosec,
                         clock_type=self.get_clock().clock_type)
        time_received = self.get_clock().now()

        # Calculate trip time
        trip_time = time_received - time_sent

        self.get_logger().info(
            f"Heartbeat received from GUI after {str(trip_time)}")

        # Send a heartbeat back
        self.send_hearbeat()

    def send_hearbeat(self):
        msg = Heartbeat()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.entity_name = SELF_ENTITY_NAME

        self.heartbeat_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    core = Core()

    rclpy.spin(core)

    core.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
