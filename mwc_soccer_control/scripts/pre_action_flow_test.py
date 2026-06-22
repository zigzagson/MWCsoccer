#!/usr/bin/env python3

import argparse
import sys
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from soccer_msgs.msg import BehaviorState, GameModeCommand, NavStatus, SoccerPerception, VisionTrackCommand


@dataclass
class BallPose:
    x: float
    y: float
    z: float = 0.0

    def x_y_z(self) -> tuple:
        return self.x, self.y, self.z


class PreActionFlowTestNode(Node):
    def __init__(self, sim_align_ball_x_m: float, sim_align_ball_y_m: float,
                 sim_settle_ball_x_m: float, sim_settle_ball_y_m: float) -> None:
        super().__init__("pre_action_flow_test")

        self.declare_parameter("timeout_s", 20.0)
        self.declare_parameter("nav_reached_after_s", 1.2)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("sim_align_ball_x_m", 0.32)
        self.declare_parameter("sim_align_ball_y_m", -0.15)
        self.declare_parameter("sim_settle_ball_x_m", 0.22)
        self.declare_parameter("sim_settle_ball_y_m", 0.0)

        self.timeout_s = float(self.get_parameter("timeout_s").value)
        self.nav_reached_after_s = float(self.get_parameter("nav_reached_after_s").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.sim_align_ball_x_m = sim_align_ball_x_m
        self.sim_align_ball_y_m = sim_align_ball_y_m
        self.sim_settle_ball_x_m = sim_settle_ball_x_m
        self.sim_settle_ball_y_m = sim_settle_ball_y_m

        durable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.game_mode_pub = self.create_publisher(
            GameModeCommand, "/soccer/game_mode_cmd", durable_qos
        )
        self.nav_status_pub = self.create_publisher(NavStatus, "/soccer/nav_status", durable_qos)
        self.perception_pub = self.create_publisher(SoccerPerception, "/soccer/perception", reliable_qos)

        self.behavior_sub = self.create_subscription(
            BehaviorState,
            "/soccer/behavior_state",
            self.on_behavior_state,
            durable_qos,
        )
        self.vision_sub = self.create_subscription(
            VisionTrackCommand,
            "/soccer/vision_track_cmd",
            self.on_vision_cmd,
            reliable_qos,
        )

        self.start_time = self.get_clock().now()
        self.last_publish_time = self.start_time
        self.last_behavior: Optional[str] = None
        self.behavior_history: List[str] = []
        self.last_vision_cmd: Optional[str] = None
        self.finish_seen = False
        self.penalty_seen = False
        self.start_track_seen = False
        self.align_seen = False
        self.ready_kick_seen = False
        self.done = False

        period = 1.0 / max(1.0, self.publish_rate_hz)
        self.timer = self.create_timer(period, self.tick)

        self.get_logger().info("pre_action_flow_test node started")

    def on_behavior_state(self, msg: BehaviorState) -> None:
        if msg.state != self.last_behavior:
            self.last_behavior = msg.state
            self.behavior_history.append(msg.state)
            self.get_logger().info(f"behavior_state -> {msg.mode}/{msg.state}: {msg.detail}")

        if msg.mode == "PENALTY_ATTACK":
            self.penalty_seen = True
        if msg.state == "START_BALL_TRACK":
            self.start_track_seen = True
        if msg.state == "ALIGN":
            self.align_seen = True
        if msg.state == "READY_KICK":
            self.ready_kick_seen = True
        if msg.state == "FINISH":
            self.finish_seen = True

    def on_vision_cmd(self, msg: VisionTrackCommand) -> None:
        self.last_vision_cmd = msg.command
        self.get_logger().info(f"vision_track_cmd -> {msg.command}")

    def tick(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.timeout_s:
            self.get_logger().error(
                f"timeout after {self.timeout_s:.1f}s; history={self.behavior_history}"
            )
            self.done = True
            self.timer.cancel()
            return

        self.publish_game_mode()
        self.publish_nav_status(elapsed)
        self.publish_perception(elapsed)

        if self.finish_seen:
            expected = [
                "NAVIGATE_TO_POINT",
                "START_BALL_TRACK",
                "ALIGN",
                "STOP_BALL_TRACK",
                "READY_KICK",
                "FINISH",
            ]
            if self._contains_sequence(expected):
                self.get_logger().info("pre-action flow closed loop passed")
                self.done = True
                self.timer.cancel()
            else:
                self.get_logger().error(
                    f"finished with unexpected behavior history={self.behavior_history}"
                )
                self.done = True
                self.timer.cancel()

    def publish_game_mode(self) -> None:
        msg = GameModeCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = "PENALTY_ATTACK"
        msg.goal_target = 0.0
        msg.nav_ball_distance_m = 0.75
        msg.reset_state = True
        self.game_mode_pub.publish(msg)

    def publish_nav_status(self, elapsed: float) -> None:
        msg = NavStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = "PENALTY_ATTACK"
        msg.nav_alive = True
        msg.perception_alive = True
        msg.navigating_to_point = elapsed < self.nav_reached_after_s
        msg.target_reached = elapsed >= self.nav_reached_after_s
        msg.detail = "simulated nav status"
        self.nav_status_pub.publish(msg)

    def publish_perception(self, elapsed: float) -> None:
        msg = SoccerPerception()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.image_has_ball = elapsed >= self.nav_reached_after_s + 0.25
        msg.image_has_goal = False
        msg.ball = self._point_stamp("dummy", *self._ball_pose(elapsed).x_y_z())
        msg.goal_center = self._point_stamp("dummy", 1.0, 0.0, 0.0)
        msg.left_post = self._point_stamp("dummy", 1.0, 0.4, 0.0)
        msg.right_post = self._point_stamp("dummy", 1.0, -0.4, 0.0)
        msg.ball_confidence = 0.95 if msg.image_has_ball else 0.0
        msg.goal_confidence = 0.0
        msg.transform_valid = msg.image_has_ball
        msg.detail = "simulated ball perception"
        self.perception_pub.publish(msg)

    def _ball_pose(self, elapsed: float) -> BallPose:
        if elapsed < self.nav_reached_after_s + 0.8:
            return BallPose(self.sim_align_ball_x_m, self.sim_align_ball_y_m, 0.0)
        if elapsed < self.nav_reached_after_s + 1.6:
            return BallPose(
                (self.sim_align_ball_x_m + self.sim_settle_ball_x_m) / 2.0,
                (self.sim_align_ball_y_m + self.sim_settle_ball_y_m) / 2.0,
                0.0,
            )
        return BallPose(self.sim_settle_ball_x_m, self.sim_settle_ball_y_m, 0.0)

    def _point_stamp(self, frame_id: str, x: float, y: float, z: float) -> PointStamped:
        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = frame_id
        point.point.x = float(x)
        point.point.y = float(y)
        point.point.z = float(z)
        return point

    def _contains_sequence(self, expected: List[str]) -> bool:
        if not self.behavior_history:
            return False
        idx = 0
        for state in self.behavior_history:
            if idx < len(expected) and state == expected[idx]:
                idx += 1
        return idx == len(expected)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim-align-ball-x-m", type=float, default=0.32)
    parser.add_argument("--sim-align-ball-y-m", type=float, default=-0.15)
    parser.add_argument("--sim-settle-ball-x-m", type=float, default=0.22)
    parser.add_argument("--sim-settle-ball-y-m", type=float, default=0.0)
    parsed, remaining = parser.parse_known_args()

    rclpy.init(args=remaining)
    node = PreActionFlowTestNode(
        parsed.sim_align_ball_x_m,
        parsed.sim_align_ball_y_m,
        parsed.sim_settle_ball_x_m,
        parsed.sim_settle_ball_y_m,
    )
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
