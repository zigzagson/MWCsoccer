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
        self.right_seen = False
        self.left_seen = False
        self.center_seen = False
        self.done = False
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(
            "goalkeeper test started: center -> right offset -> center -> "
            "invalid perception -> left offset -> center"
        )

    def on_behavior(self, msg: BehaviorState) -> None:
        if msg.state != self.last_state:
            self.last_state = msg.state
            self.history.append(msg.state)
            self.get_logger().info(
                f"behavior_state -> {msg.mode}/{msg.state}: {msg.detail}"
            )

        if "goalkeeper control" not in msg.detail or "vy=" not in msg.detail:
            return
        try:
            vy = float(msg.detail.split("vy=", 1)[1].split()[0])
        except ValueError:
            return
        if vy > 0.0:
            self.right_seen = True
        elif vy < 0.0:
            self.left_seen = True
        else:
            self.center_seen = True

    def tick(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed >= 7.0:
            if self.right_seen and self.left_seen and self.center_seen:
                self.get_logger().info(
                    "goalkeeper flow passed: right, left and center commands "
                    f"observed; history={self.history}"
                )
            else:
                self.get_logger().error(
                    "goalkeeper flow failed: "
                    f"right={self.right_seen} left={self.left_seen} "
                    f"center={self.center_seen} "
                    f"history={self.history}"
                )
            self.done = True
            self.timer.cancel()
            return

        valid = True
        x = 0.0
        y = 0.20
        if 1.0 <= elapsed < 2.0:
            x = 0.40
        elif 3.0 <= elapsed < 3.6:
            valid = False
        elif 4.5 <= elapsed < 5.5:
            x = -0.40

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
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
