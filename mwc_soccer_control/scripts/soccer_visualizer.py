#!/usr/bin/env python3

import json
import os
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from soccer_msgs.msg import (
    BehaviorState,
    GameModeCommand,
    NavStatus,
    SoccerPerception,
)


def message_time(header: Any) -> float:
    return float(header.stamp.sec) + float(header.stamp.nanosec) / 1e9


class RealtimeState:
    def __init__(self, config: Dict[str, float]) -> None:
        self.lock = threading.Lock()
        self.sequence = 0
        self.config = config
        self.data: Dict[str, Any] = {
            "behavior": None,
            "perception": None,
            "navigation": None,
            "velocity": None,
            "command": None,
            "events": [],
        }
        self.received_at: Dict[str, float] = {}

    def update(self, key: str, value: Dict[str, Any]) -> None:
        with self.lock:
            self.sequence += 1
            self.data[key] = value
            self.received_at[key] = time.monotonic()

    def add_event(self, mode: str, state: str, detail: str) -> None:
        event = {
            "time": time.strftime("%H:%M:%S"),
            "mode": mode,
            "state": state,
            "detail": detail,
        }
        with self.lock:
            events = self.data["events"]
            if events and events[-1]["mode"] == mode and events[-1]["state"] == state:
                events[-1] = event
            else:
                events.append(event)
                del events[:-16]

    def snapshot(self) -> Dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            payload = {
                "sequence": self.sequence,
                "server_time": time.time(),
                "config": dict(self.config),
                "events": list(self.data["events"]),
            }
            for key in ("behavior", "perception", "navigation", "velocity", "command"):
                value = self.data[key]
                payload[key] = dict(value) if value is not None else None
                payload[f"{key}_age_s"] = (
                    round(now - self.received_at[key], 3)
                    if key in self.received_at
                    else None
                )
            return payload


class DashboardHandler(BaseHTTPRequestHandler):
    server_version = "MWCSoccerVisualizer/1.0"

    def do_GET(self) -> None:
        if self.path == "/events":
            self.stream_events()
            return

        static_files = {
            "/": ("index.html", "text/html; charset=utf-8"),
            "/index.html": ("index.html", "text/html; charset=utf-8"),
            "/app.js": ("app.js", "text/javascript; charset=utf-8"),
            "/style.css": ("style.css", "text/css; charset=utf-8"),
        }
        entry = static_files.get(self.path.split("?", 1)[0])
        if entry is None:
            self.send_error(404)
            return

        filename, content_type = entry
        path = os.path.join(self.server.web_root, filename)  # type: ignore[attr-defined]
        try:
            with open(path, "rb") as source:
                content = source.read()
        except OSError:
            self.send_error(500, "dashboard asset unavailable")
            return

        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(content)

    def stream_events(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()

        interval = 1.0 / self.server.refresh_hz  # type: ignore[attr-defined]
        try:
            while not self.server.stop_event.is_set():  # type: ignore[attr-defined]
                state = self.server.realtime_state.snapshot()  # type: ignore[attr-defined]
                payload = json.dumps(
                    state, ensure_ascii=False, separators=(",", ":")
                )
                self.wfile.write(f"data:{payload}\n\n".encode("utf-8"))
                self.wfile.flush()
                time.sleep(interval)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, fmt: str, *args: Any) -> None:
        if self.path != "/events":
            super().log_message(fmt, *args)


class DashboardServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(
        self,
        address: tuple,
        web_root: str,
        realtime_state: RealtimeState,
        refresh_hz: float,
    ) -> None:
        super().__init__(address, DashboardHandler)
        self.web_root = web_root
        self.realtime_state = realtime_state
        self.refresh_hz = refresh_hz
        self.stop_event = threading.Event()


class SoccerVisualizer(Node):
    DETAIL_VALUE = re.compile(
        r"(?P<key>[A-Za-z_]+)=(?P<value>[-+]?(?:\d+\.?\d*|\.\d+))"
    )
    CELEBRATION_SELECTED = re.compile(
        r"(?:^|\s)celebration_selected=(?P<value>[A-Za-z0-9_]+)"
    )

    def __init__(self) -> None:
        super().__init__("soccer_visualizer")
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("port", 18080)
        self.declare_parameter("refresh_hz", 10.0)
        self.declare_parameter("velocity_command_topic", "/nav/cmd_vel_nav")
        self.declare_parameter("align_target_ball_x_m", 0.8)
        self.declare_parameter("align_target_ball_y_m", -0.3)
        self.declare_parameter("align_x_tolerance_m", 0.1)
        self.declare_parameter("align_y_tolerance_m", 0.07)
        self.declare_parameter("goalkeeper_center_deadband_x", 0.05)
        self.declare_parameter("perception_stale_s", 1.0)
        self.declare_parameter("nav_stale_s", 2.0)

        config = {
            "align_target_ball_x_m": self.param_float("align_target_ball_x_m"),
            "align_target_ball_y_m": self.param_float("align_target_ball_y_m"),
            "align_x_tolerance_m": self.param_float("align_x_tolerance_m"),
            "align_y_tolerance_m": self.param_float("align_y_tolerance_m"),
            "goalkeeper_center_deadband_x": self.param_float(
                "goalkeeper_center_deadband_x"
            ),
            "perception_stale_s": self.param_float("perception_stale_s"),
            "nav_stale_s": self.param_float("nav_stale_s"),
        }
        self.state = RealtimeState(config)
        self.last_behavior_key: Optional[tuple] = None

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

        self.create_subscription(
            BehaviorState,
            "/soccer/behavior_state",
            self.on_behavior,
            durable_qos,
        )
        self.create_subscription(
            SoccerPerception,
            "/soccer/perception",
            self.on_perception,
            reliable_qos,
        )
        self.create_subscription(
            NavStatus,
            "/soccer/nav_status",
            self.on_navigation,
            durable_qos,
        )
        self.create_subscription(
            GameModeCommand,
            "/soccer/game_mode_cmd",
            self.on_command,
            durable_qos,
        )
        velocity_topic = self.get_parameter("velocity_command_topic").value
        self.create_subscription(
            Twist,
            str(velocity_topic),
            self.on_velocity,
            reliable_qos,
        )

        bind_address = str(self.get_parameter("bind_address").value)
        port = int(self.get_parameter("port").value)
        refresh_hz = max(1.0, self.param_float("refresh_hz"))
        web_root = os.path.join(
            get_package_share_directory("mwc_soccer_control"), "web"
        )
        self.server = DashboardServer(
            (bind_address, port), web_root, self.state, refresh_hz
        )
        self.server_thread = threading.Thread(
            target=self.server.serve_forever,
            name="soccer-dashboard-http",
            daemon=True,
        )
        self.server_thread.start()
        shown_host = "127.0.0.1" if bind_address == "0.0.0.0" else bind_address
        self.get_logger().info(
            f"soccer dashboard listening on http://{shown_host}:{port}"
        )

    def param_float(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def on_behavior(self, msg: BehaviorState) -> None:
        values = {
            match.group("key"): float(match.group("value"))
            for match in self.DETAIL_VALUE.finditer(msg.detail)
        }
        selected_match = self.CELEBRATION_SELECTED.search(msg.detail)
        self.state.update(
            "behavior",
            {
                "stamp": message_time(msg.header),
                "mode": msg.mode,
                "state": msg.state,
                "detail": msg.detail,
                "progress": round(float(msg.progress), 4),
                "active": bool(msg.active),
                "values": values,
                "celebration_selected": (
                    selected_match.group("value") if selected_match else None
                ),
            },
        )
        behavior_key = (msg.mode, msg.state)
        if behavior_key != self.last_behavior_key:
            self.last_behavior_key = behavior_key
            self.state.add_event(msg.mode, msg.state, msg.detail)

    def on_perception(self, msg: SoccerPerception) -> None:
        def point(value: Any) -> Dict[str, Any]:
            return {
                "frame": value.header.frame_id,
                "x": round(float(value.point.x), 4),
                "y": round(float(value.point.y), 4),
                "z": round(float(value.point.z), 4),
            }

        self.state.update(
            "perception",
            {
                "stamp": message_time(msg.header),
                "image_has_ball": bool(msg.image_has_ball),
                "image_has_goal": bool(msg.image_has_goal),
                "transform_valid": bool(msg.transform_valid),
                "ball_confidence": round(float(msg.ball_confidence), 4),
                "goal_confidence": round(float(msg.goal_confidence), 4),
                "ball": point(msg.ball),
                "goal_center": point(msg.goal_center),
                "left_post": point(msg.left_post),
                "right_post": point(msg.right_post),
                "detail": msg.detail,
            },
        )

    def on_navigation(self, msg: NavStatus) -> None:
        self.state.update(
            "navigation",
            {
                "stamp": message_time(msg.header),
                "mode": msg.mode,
                "nav_alive": bool(msg.nav_alive),
                "perception_alive": bool(msg.perception_alive),
                "navigating_to_point": bool(msg.navigating_to_point),
                "target_reached": bool(msg.target_reached),
                "detail": msg.detail,
            },
        )

    def on_command(self, msg: GameModeCommand) -> None:
        self.state.update(
            "command",
            {
                "stamp": message_time(msg.header),
                "mode": msg.mode,
                "goal_target": round(float(msg.goal_target), 4),
                "nav_ball_distance_m": round(float(msg.nav_ball_distance_m), 4),
                "reset_state": bool(msg.reset_state),
            },
        )

    def on_velocity(self, msg: Twist) -> None:
        self.state.update(
            "velocity",
            {
                "vx": round(float(msg.linear.x), 4),
                "vy": round(float(msg.linear.y), 4),
                "vz": round(float(msg.linear.z), 4),
                "wz": round(float(msg.angular.z), 4),
                "stop": bool(msg.linear.z > 0.5),
            },
        )

    def destroy_node(self) -> bool:
        self.server.stop_event.set()
        self.server.shutdown()
        self.server.server_close()
        self.server_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = SoccerVisualizer()
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
