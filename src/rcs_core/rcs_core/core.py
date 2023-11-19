import rclpy
from rclpy.node import Node, Subscription, Timer
from rclpy.clock import Time, Duration

from rcs_interfaces.msg import Heartbeat


ENTITY_NAME = "rcs_core"

HEARTBEAT_TIMER_INTERVAL = 5.0
"""About how long to wait between expected heartbeats."""

HEARTBEAT_CHECK_INTERVAL = 1.0
"""How often to check for missed heartbeats."""

HEARTBEAT_GRACE_PERIOD = 3.0
"""How long to wait after the expected heartbeat before declaring a failure."""

COMPONENT_STATUS_INACTIVE = "inactive"
COMPONENT_STATUS_ACTIVE = "active"
COMPONENT_STATUS_FAILED = "failed"


class Core(Node):
    heartbeat_times: dict[str, Time] = {}
    component_statuses: dict[str, str] = {"tester": "inactive"}
    heartbeat_timer: Timer
    heartbeat_sub: Subscription

    def __init__(self):
        super().__init__(ENTITY_NAME)

        self.heartbeat_timer = self.create_timer(
            HEARTBEAT_CHECK_INTERVAL, self.heartbeat_timer_callback)

        self.heartbeat_sub = self.create_subscription(
            Heartbeat, f"{ENTITY_NAME}/heartbeat/sub", self.heartbeat_callback, 10)

        self.get_logger().info("RCS Core initialized")

    def heartbeat_timer_callback(self):
        # Check for missed heartbeats
        now = self.get_clock().now()

        for node_name, last_heartbeat in self.heartbeat_times.items():
            # Check if the heartbeat is too old
            if (now - last_heartbeat) > Duration(seconds=HEARTBEAT_TIMER_INTERVAL + HEARTBEAT_GRACE_PERIOD):
                if self.component_statuses[node_name] != COMPONENT_STATUS_ACTIVE:
                    continue

                # If it is, log the error
                self.get_logger().error(f"Missed heartbeat from {node_name}!")
                # Mark the component as failed
                self.component_statuses[node_name] = COMPONENT_STATUS_FAILED

    def heartbeat_callback(self, msg: Heartbeat):
        if msg.node_name not in self.component_statuses:
            self.get_logger().warning(
                f"Unexpected heartbeat from {msg.node_name}")
            return

        if self.component_statuses[msg.node_name] == COMPONENT_STATUS_FAILED:
            self.get_logger().info(f"Reconnected to {msg.node_name}")
        elif self.component_statuses[msg.node_name] == COMPONENT_STATUS_INACTIVE:
            self.get_logger().info(f"Connected to {msg.node_name}")

        # Log the time we received the heartbeat
        self.heartbeat_times[msg.node_name] = self.get_clock().now()
        # Make sure we mark the component as active
        self.component_statuses[msg.node_name] = COMPONENT_STATUS_ACTIVE


def main(args=None):
    rclpy.init(args=args)

    core = Core()

    rclpy.spin(core)

    core.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
