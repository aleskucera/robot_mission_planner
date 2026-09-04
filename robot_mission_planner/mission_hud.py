#!/usr/bin/env python3
"""Mission HUD: what an operator watches during a Robotour run, as rviz overlays.

Collects the mission and robot topics into two rviz_2d_overlay_msgs/OverlayText
panels drawn over the 3D view -- ``~/mission`` (follower state, route progress,
commander, last event, planner, QR goal) top left and ``~/status`` (e-stop,
battery, motor temperatures, GNSS fix) top right -- plus ``~/robot_body``, a
placeholder box marker for replays where nothing publishes /robot_description.

Panel size, position and colours travel inside the OverlayText message, so the
rviz displays only need the topic; leave their "Overtake Position Properties"
and "Overtake ... Color Properties" switches off unless you want to move a panel
from rviz itself.

Every input is a parameter, so the same node serves the robot and a bag replay.
TF is always looked up at time zero (= latest available): a bag replays with its
original stamps, which are nowhere near the node's wall clock.
"""

from __future__ import annotations

import math

import rclpy
from geographic_msgs.msg import GeoPointStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from rviz_2d_overlay_msgs.msg import OverlayText
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus, Temperature
from std_msgs.msg import Bool, ColorRGBA, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

# Latched publishers (road_follower state/event, route_planner status/route_path) offer
# RELIABLE + TRANSIENT_LOCAL; everything else is subscribed BEST_EFFORT, which a reliable
# publisher also satisfies, so a bag replay connects whatever QoS it was recorded with.
LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
PLAIN = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)

GREY = "#9aa0a6"
GREEN = "#5bd75b"
AMBER = "#ffb74d"
RED = "#ff5252"
BLUE = "#64b5f6"
CYAN = "#4dd0e1"
WHITE = "#e8eaed"

# road_follower states as they arrive on ~/state ("GPS:<reason>" carries a suffix).
STATE_COLORS = {
    "ROAD": GREEN,
    "GPS": AMBER,
    "PLANNING": BLUE,
    "ARRIVED": CYAN,
    "IDLE": GREY,
}

FIX_STATUS = {
    NavSatStatus.STATUS_NO_FIX: ("no fix", RED),
    NavSatStatus.STATUS_FIX: ("fix", GREEN),
    NavSatStatus.STATUS_SBAS_FIX: ("SBAS fix", GREEN),
    NavSatStatus.STATUS_GBAS_FIX: ("GBAS fix", GREEN),
}


def rgba(r: float, g: float, b: float, a: float) -> ColorRGBA:
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


class MissionHud(Node):
    def __init__(self) -> None:
        super().__init__("mission_hud")
        p = self.declare_parameter

        self.map_frame = p("map_frame", "FP_ENU0").value
        self.robot_frame = p("robot_frame", "base_link").value

        # ---- inputs (empty topic = that line is simply never filled in)
        follower_state = p("follower_state_topic", "/road_follower/state").value
        follower_event = p("follower_event_topic", "/road_follower/event").value
        commander_state = p("commander_state_topic", "/crl_commander/state").value
        planner_status = p("planner_status_topic", "/route_planner/status").value
        route_path = p("route_path_topic", "/route_planner/route_path").value
        qr_goal = p("qr_goal_topic", "/qr_goal/goal").value
        estop = p("estop_topic", "/estop_active").value
        battery = p("battery_topic", "/battery_state").value
        temperature = p("temperature_topic", "/temperatures").value
        gps_fix = p("gps_fix_topic", "/fixposition/odometry_llh").value

        # ---- panel look. Sizes are in pixels of the 3D render panel.
        self.text_size = float(p("text_size", 12.0).value)
        self.panel_width = int(p("panel_width", 620).value)
        # Lines are truncated rather than wrapped: a wrapped line would spill out of the
        # box, which is sized from the line count.
        self.max_chars = int(p("panel_max_chars", 60).value)
        self.margin = int(p("panel_margin", 8).value)
        self.bg_alpha = float(p("panel_bg_alpha", 0.55).value)
        self.font = p("font", "DejaVu Sans Mono").value
        # QStaticText renders rich text, so the panels colour single values. Set false if a
        # future rviz build draws the markup verbatim.
        self.markup = bool(p("markup", True).value)
        self.rate = float(p("rate", 4.0).value)

        # ---- placeholder robot body (a URDF on /robot_description is the real thing)
        self.body_enabled = bool(p("robot_body", True).value)
        self.body_size = [float(v) for v in p("robot_body_size", [0.9, 0.7, 0.35]).value]
        self.body_offset_z = float(p("robot_body_offset_z", 0.25).value)
        self.body_color = [float(v) for v in p("robot_body_color", [0.25, 0.6, 1.0, 0.5]).value]

        self._follower = ""
        self._event = ""
        self._event_at = None
        self._commander = ""
        self._planner = ""
        self._route: Path | None = None
        self._goal: GeoPointStamped | None = None
        self._estop: bool | None = None
        self._battery: BatteryState | None = None
        self._temps: dict[str, float] = {}
        self._fix: NavSatFix | None = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        def sub(topic, msg_type, cb, qos):
            if topic:
                self.create_subscription(msg_type, topic, cb, qos)

        sub(follower_state, String, lambda m: setattr(self, "_follower", m.data), LATCHED)
        sub(follower_event, String, self._event_cb, LATCHED)
        sub(commander_state, String, lambda m: setattr(self, "_commander", m.data), PLAIN)
        sub(planner_status, String, lambda m: setattr(self, "_planner", m.data), LATCHED)
        sub(route_path, Path, lambda m: setattr(self, "_route", m), LATCHED)
        sub(qr_goal, GeoPointStamped, lambda m: setattr(self, "_goal", m), LATCHED)
        sub(estop, Bool, lambda m: setattr(self, "_estop", m.data), PLAIN)
        sub(battery, BatteryState, lambda m: setattr(self, "_battery", m), PLAIN)
        sub(temperature, Temperature, self._temperature_cb, PLAIN)
        sub(gps_fix, NavSatFix, lambda m: setattr(self, "_fix", m), PLAIN)

        self.pub_mission = self.create_publisher(OverlayText, "~/mission", 1)
        self.pub_status = self.create_publisher(OverlayText, "~/status", 1)
        self.pub_body = self.create_publisher(MarkerArray, "~/robot_body", 1)

        self.create_timer(1.0 / max(self.rate, 0.1), self._tick)
        if self.body_enabled:
            self.create_timer(1.0, self._publish_body)
        self.get_logger().info(
            f"mission_hud: panels on {self.pub_mission.topic_name} and {self.pub_status.topic_name}"
        )

    # ------------------------------------------------------------------ callbacks
    def _event_cb(self, msg: String) -> None:
        self._event = msg.data
        self._event_at = self.get_clock().now()

    def _temperature_cb(self, msg: Temperature) -> None:
        self._temps[msg.header.frame_id or "?"] = msg.temperature

    # ------------------------------------------------------------------ helpers
    def _color(self, text: str, color: str) -> str:
        return f'<span style="color:{color};">{text}</span>' if self.markup else text

    def _row(self, label: str, value: str, color: str = WHITE) -> str:
        room = max(self.max_chars - 10, 8)
        if len(value) > room:
            value = value[: room - 1] + "\u2026"
        return f'{self._color(f"{label:<10}", GREY)}{self._color(value, color)}'

    def _robot_xy(self, frame: str) -> tuple[float, float] | None:
        try:
            tr = self.tf_buffer.lookup_transform(frame, self.robot_frame, Time()).transform
        except Exception:
            return None
        return tr.translation.x, tr.translation.y

    def _route_progress(self) -> str:
        route = self._route
        if route is None or not route.poses:
            return "-"
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in route.poses]
        total = sum(math.dist(pts[k], pts[k + 1]) for k in range(len(pts) - 1))
        here = self._robot_xy(route.header.frame_id or self.map_frame)
        if here is None:
            return f"{len(pts)} wp, {total:.0f} m (no robot tf)"
        # Nearest waypoint, then the route length left from it to the end.
        i = min(range(len(pts)), key=lambda k: math.dist(pts[k], here))
        left = math.dist(here, pts[i]) + sum(math.dist(pts[k], pts[k + 1]) for k in range(i, len(pts) - 1))
        return f"{i + 1}/{len(pts)} wp   {left:.0f} m left of {total:.0f} m"

    def _overlay(self, lines: list[str], right: bool, bg: ColorRGBA) -> OverlayText:
        msg = OverlayText()
        msg.action = OverlayText.ADD
        msg.width = self.panel_width
        # Grow the box with the content instead of leaving a large empty rectangle.
        msg.height = int(round(len(lines) * self.text_size * 1.6 + 14))
        msg.horizontal_alignment = OverlayText.RIGHT if right else OverlayText.LEFT
        msg.vertical_alignment = OverlayText.TOP
        msg.horizontal_distance = self.margin
        msg.vertical_distance = self.margin
        msg.bg_color = bg
        msg.fg_color = rgba(0.91, 0.92, 0.93, 1.0)
        msg.line_width = 2
        msg.text_size = self.text_size
        msg.font = self.font
        msg.text = "<br>".join(lines) if self.markup else "\n".join(lines)
        return msg

    # ------------------------------------------------------------------ panels
    def _mission_panel(self) -> OverlayText:
        state = self._follower or "?"
        color = STATE_COLORS.get(state.split(":")[0], WHITE)
        lines = [self._row("FOLLOWER", state, color)]

        lines.append(self._row("route", self._route_progress()))

        commander = self._commander or "-"
        lines.append(self._row("commander", commander, AMBER if "STUCK" in commander.upper() else WHITE))

        event = self._event or "-"
        if self._event_at is not None:
            age = (self.get_clock().now() - self._event_at).nanoseconds / 1e9
            event = f"{event}  ({age:.0f} s ago)"
        lines.append(self._row("event", event))

        planner = self._planner or "-"
        lines.append(self._row("planner", planner, RED if planner.startswith("failed") else WHITE))

        goal = "-"
        if self._goal is not None:
            goal = f"{self._goal.position.latitude:.7f}, {self._goal.position.longitude:.7f}"
        lines.append(self._row("QR goal", goal))
        return self._overlay(lines, right=False, bg=rgba(0.06, 0.06, 0.08, self.bg_alpha))

    def _status_panel(self) -> OverlayText:
        if self._estop is None:
            estop, estop_color = "unknown", GREY
        elif self._estop:
            estop, estop_color = "PRESSED", RED
        else:
            estop, estop_color = "released", GREEN
        lines = [self._row("E-STOP", estop, estop_color)]

        battery = "-"
        color = WHITE
        if self._battery is not None:
            b = self._battery
            # The Helhest BMS reports 0..100 here; the message spec says 0..1.
            pct = b.percentage * (1.0 if b.percentage > 1.5 else 100.0)
            color = RED if pct < 15 else AMBER if pct < 30 else GREEN
            battery = f"{pct:.0f} %   {b.voltage:.1f} V   {b.current:+.1f} A"
        lines.append(self._row("battery", battery, color))

        temps = "-"
        color = WHITE
        if self._temps:
            name, value = max(self._temps.items(), key=lambda kv: kv[1])
            color = RED if value > 80 else AMBER if value > 65 else WHITE
            temps = f"{value:.0f} C  {name}   (max of {len(self._temps)})"
        lines.append(self._row("temps", temps, color))

        gnss, fix, color = "-", "-", WHITE
        if self._fix is not None:
            f = self._fix
            fix, color = FIX_STATUS.get(f.status.status, ("?", AMBER))
            if f.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
                fix += f"   +/-{math.sqrt(max(f.position_covariance[0], 0.0)):.2f} m"
            gnss = f"{f.latitude:.7f}, {f.longitude:.7f}"
        lines.append(self._row("GNSS", gnss))
        lines.append(self._row("fix", fix, color))

        # The whole panel goes dark red while the e-stop is in, so it reads across the room.
        bg = rgba(0.35, 0.02, 0.02, 0.75) if self._estop else rgba(0.06, 0.06, 0.08, self.bg_alpha)
        return self._overlay(lines, right=True, bg=bg)

    def _tick(self) -> None:
        self.pub_mission.publish(self._mission_panel())
        self.pub_status.publish(self._status_panel())

    def _publish_body(self) -> None:
        """A box where the robot is, so a replay without /robot_description shows something."""
        now = self.get_clock().now().to_msg()
        body = Marker()
        body.header.frame_id = self.robot_frame
        body.header.stamp = now
        body.ns = "robot_body"
        body.id = 0
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.pose.position.z = self.body_offset_z
        body.pose.orientation.w = 1.0
        body.scale.x, body.scale.y, body.scale.z = self.body_size
        body.color = rgba(*self.body_color)

        heading = Marker()
        heading.header = body.header
        heading.ns = "robot_body"
        heading.id = 1
        heading.type = Marker.ARROW
        heading.action = Marker.ADD
        heading.pose.position.z = self.body_offset_z
        heading.pose.orientation.w = 1.0
        heading.scale.x = self.body_size[0] * 1.4  # length
        heading.scale.y = 0.12
        heading.scale.z = 0.12
        heading.color = rgba(1.0, 0.85, 0.2, 0.9)
        self.pub_body.publish(MarkerArray(markers=[body, heading]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionHud()
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
