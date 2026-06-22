#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from soccer_msgs.msg import BehaviorState


class CelebrationSelectionTest(Node):
    ACTIONS = (
        "neymar_victory_dance",
        "forward_jump",
        "raised_hand_taunt",
        "stretch_wave",
    )

    def __init__(self) -> None:
        super().__init__("celebration_selection_test")
        self.declare_parameter("interval_s", 2.0)
        interval_s = max(0.2, float(self.get_parameter("interval_s").value))

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            BehaviorState, "/soccer/behavior_state", qos
        )
        self.index = 0
        self.publish_selection()
        self.timer = self.create_timer(interval_s, self.select_next)
        self.get_logger().info(
            "display-only celebration selection test started; "
            "no whole-body action will be sent"
        )

    def select_next(self) -> None:
        self.index = (self.index + 1) % len(self.ACTIONS)
        self.publish_selection()

    def publish_selection(self) -> None:
        action = self.ACTIONS[self.index]
        msg = BehaviorState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = "IDLE"
        msg.state = "IDLE"
        msg.detail = (
            f"display-only selection test celebration_selected={action}"
        )
        msg.progress = 0.0
        msg.active = False
        self.publisher.publish(msg)
        self.get_logger().info(
            f"selected celebration: {action} "
            f"({self.index + 1}/{len(self.ACTIONS)})"
        )


def main() -> None:
    rclpy.init()
    node = CelebrationSelectionTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
