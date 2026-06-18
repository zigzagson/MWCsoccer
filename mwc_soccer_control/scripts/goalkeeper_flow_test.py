#!/usr/bin/env python3

import sys
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from soccer_msgs.msg import BehaviorState, SoccerPerception


class GoalkeeperFlowTest(Node):
    def __init__(self) -> None:
        super().__init__("goalkeeper_flow_test")
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        durable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.perception_pub = self.create_publisher(
            SoccerPerception, "/soccer/perception", reliable_qos
        )
        self.behavior_sub = self.create_subscription(
            BehaviorState,
            "/soccer/behavior_state",
            self.on_behavior,
            durable_qos,
        )
        self.start_time = self.get_clock().now()
        self.last_state: Optional[str] = None
        self.history: List[str] = []
        self.move_count = 0
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(
            "goalkeeper test started: stationary -> right shot -> invalid "
            "perception -> left shot"
        )

    def on_behavior(self, msg: BehaviorState) -> None:
        if msg.state == self.last_state:
            return
        self.last_state = msg.state
        self.history.append(msg.state)
        self.get_logger().info(
            f"behavior_state -> {msg.mode}/{msg.state}: {msg.detail}"
        )
        if msg.state == "GOALKEEPER_MOVE":
            self.move_count += 1

    def tick(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed >= 7.0:
            if self.move_count >= 2:
                self.get_logger().info(
                    f"goalkeeper flow passed: move_count={self.move_count} "
                    f"history={self.history}"
                )
            else:
                self.get_logger().error(
                    f"goalkeeper flow failed: move_count={self.move_count} "
                    f"history={self.history}"
                )
            rclpy.shutdown()
            return

        valid = True
        x = 0.0
        y = 0.20
        if 1.0 <= elapsed < 1.4:
            x = (elapsed - 1.0) * 1.8
            y = 0.20 - (elapsed - 1.0) * 0.4
        elif 3.0 <= elapsed < 3.6:
            valid = False
        elif 4.5 <= elapsed < 4.9:
            x = -(elapsed - 4.5) * 1.8
            y = 0.20 - (elapsed - 4.5) * 0.4

        self.publish_perception(x, y, valid)

    def publish_perception(self, x: float, y: float, valid: bool) -> None:
        msg = SoccerPerception()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.image_has_ball = valid
        msg.image_has_goal = False
        msg.transform_valid = valid
        msg.ball_confidence = 0.95 if valid else 0.0
        msg.goal_confidence = 0.0
        msg.ball = self.point(x, y)
        msg.detail = "goalkeeper_flow_test valid" if valid else "simulated invalid"
        self.perception_pub.publish(msg)

    def point(self, x: float, y: float) -> PointStamped:
        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = "wide_rgb_image_center"
        point.point.x = x
        point.point.y = y
        point.point.z = 0.0
        return point


def main() -> int:
    rclpy.init(args=sys.argv)
    node = GoalkeeperFlowTest()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
