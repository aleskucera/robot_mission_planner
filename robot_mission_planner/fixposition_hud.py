#!/usr/bin/env python3
"""Fixposition HUD: is the Vision-RTK fusion healthy, as two lines of rviz overlay.

FP_A-ODOMSTATUS (``/fixposition/fpa/odomstatus``) reports two dozen sub-statuses; an
operator glances at this panel mid-mission, so it answers one question -- can the pose be
trusted -- in two lines and nothing more:

    FP global  fuse imu g1 g2 cor ws
    g1 fix  g2 fix  cor ok  imu fine  ws ok

The headline is the fusion initialisation plus the measurements the filter actually folds
in (``!`` marks a degraded one), and, while the IMU bias or the wheelspeed is still
converging, the one word saying what it waits for. The second line is the detail behind
it: both GNSS fixes, the RTK corrections, IMU bias and wheelspeed. Every value is a short
token, colour carries the verdict, and the rest of the message (cameras, markers, IMU
noise, baseline) is left to ``ros2 topic echo``.

Built like ``mission_hud`` and meant to run next to it: panel size, position and colours
travel inside the rviz_2d_overlay_msgs/OverlayText message, so the rviz display only needs
the topic, and the panel sits bottom right by default, clear of the two mission panels.

Every input is a parameter, so the same node serves the robot and a bag replay.
"""

from __future__ import annotations

import rclpy
from fixposition_driver_msgs.msg import FpaOdomstatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rviz_2d_overlay_msgs.msg import OverlayText
from std_msgs.msg import ColorRGBA

# The driver publishes at the sensor's output rate, RELIABLE; BEST_EFFORT here is satisfied
# by a reliable publisher too, so a bag replay connects whatever QoS it was recorded with.
PLAIN = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)

GREY = "#9aa0a6"
GREEN = "#5bd75b"
AMBER = "#ffb74d"
RED = "#ff5252"

# Value -> (token, colour) per field. The keys are the FpaConsts enums, spelled out rather
# than imported as names so the panel wording stays in one place; -1 (unspecified) and any
# value the sensor adds later fall back to "?" in grey. Tokens are kept to a few characters
# each: both lines have to stay inside one panel width.
INIT_STATUS = {
    0: ("NO INIT", RED),
    1: ("local", AMBER),  # pose is relative only: no global position yet
    2: ("global", GREEN),
}
GNSS_STATUS = {
    0: ("none", RED),
    1: ("spp", AMBER),
    2: ("mbase", AMBER),
    5: ("float", AMBER),
    8: ("fix", GREEN),
}
CORR_STATUS = {
    0: ("wait", GREY),
    1: ("nognss", RED),
    2: ("none", RED),
    3: ("limited", AMBER),
    4: ("old", AMBER),
    5: ("ok", GREEN),
}
IMU_STATUS = {
    0: ("none", RED),
    1: ("warm", AMBER),
    2: ("rough", AMBER),
    3: ("fine", GREEN),
}
WS_STATUS = {
    0: ("off", GREY),
    1: ("miss", RED),
    2: ("0conv", AMBER),
    3: ("1conv", AMBER),
    4: ("ok", GREEN),
}
# Why the IMU bias / the wheelspeed is not converged yet -- one of these joins the headline
# while it holds, because it is the only thing an operator can act on (drive, wait, check
# the antenna); once both are converged the headline has no such note.
IMU_CONV = {
    1: "no imu",
    2: "few meas",
    3: "no motion",
    4: "conv",
    7: "idle",
}
WS_CONV = {
    0: "no fusion",
    1: "no meas",
    2: "few meas",
    3: "no motion",
    4: "imu bias",
    5: "conv",
    6: "idle",
}

# Which measurements the filter folds in (FpaConsts.MEAS_STATUS_USED / DEGRADED), in the
# order an operator scans them. Field name -> label.
FUSION_INPUTS = [
    ("fusion_imu", "imu"),
    ("fusion_gnss1", "g1"),
    ("fusion_gnss2", "g2"),
    ("fusion_corr", "cor"),
    ("fusion_ws", "ws"),
]

SEP = "  "  # between the fields of a line


def rgba(r: float, g: float, b: float, a: float) -> ColorRGBA:
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


class FixpositionHud(Node):
    def __init__(self) -> None:
        super().__init__("fixposition_hud")
        p = self.declare_parameter

        self.topic = p("odomstatus_topic", "/fixposition/fpa/odomstatus").value
        # ODOMSTATUS is a stream: older than this [s] and the panel says so instead of
        # showing statuses that stopped being true.
        self.stale_timeout = float(p("stale_timeout", 3.0).value)

        # ---- panel look. Sizes are in pixels of the 3D render panel.
        self.text_size = float(p("text_size", 12.0).value)
        self.panel_width = int(p("panel_width", 540).value)
        # Lines are truncated rather than wrapped: a wrapped line would spill out of the
        # box, which is sized from the line count. ~9.6 px per character at text_size 12.
        self.max_chars = int(p("panel_max_chars", 54).value)
        self.margin = int(p("panel_margin", 8).value)
        self.bg_alpha = float(p("panel_bg_alpha", 0.55).value)
        # Bottom right by default: mission_hud already owns both top corners.
        self.align_right = bool(p("panel_align_right", True).value)
        self.align_bottom = bool(p("panel_align_bottom", True).value)
        self.font = p("font", "DejaVu Sans Mono").value
        # QStaticText renders rich text, so the panel colours single values. Set false if a
        # future rviz build draws the markup verbatim.
        self.markup = bool(p("markup", True).value)
        self.rate = float(p("rate", 4.0).value)

        self._msg: FpaOdomstatus | None = None
        self._msg_at = None
        self._stale = False  # set per panel, greys every value the stale message carries

        if self.topic:
            self.create_subscription(FpaOdomstatus, self.topic, self._status_cb, PLAIN)

        self.pub = self.create_publisher(OverlayText, "~/fixposition", 1)
        self.create_timer(1.0 / max(self.rate, 0.1), self._tick)
        self.get_logger().info(
            f"fixposition_hud: {self.topic or '(no topic)'} -> {self.pub.topic_name}"
        )

    # ------------------------------------------------------------------ callbacks
    def _status_cb(self, msg: FpaOdomstatus) -> None:
        self._msg = msg
        self._msg_at = self.get_clock().now()

    # ------------------------------------------------------------------ helpers
    def _color(self, text: str, color: str) -> str:
        return f'<span style="color:{color};">{text}</span>' if self.markup else text

    def _line(self, parts: list[tuple[str, str]], stale_grey: bool = True) -> str:
        """One panel line from ``(text, colour)`` pairs, truncated to the panel width.

        A stale message greys out the values it carries; the headline keeps its own
        colours, since it is the line that says the message is stale.
        """
        out, used = [], 0
        for text, color in parts:
            if not text:
                continue
            room = self.max_chars - used
            if room <= 0:
                break
            if len(text) > room:
                text = text[: max(room - 1, 1)] + "…"
            if stale_grey and self._stale:
                color = GREY
            out.append(self._color(text, color))
            used += len(text)
        return "".join(out)

    @staticmethod
    def _lookup(table: dict[int, tuple[str, str]], value: int) -> tuple[str, str]:
        return table.get(int(value), ("?", GREY))

    def _age(self) -> float | None:
        if self._msg_at is None:
            return None
        return (self.get_clock().now() - self._msg_at).nanoseconds / 1e9

    def _fusion(self, m: FpaOdomstatus) -> tuple[str, str]:
        """The measurements actually fused, degraded ones marked with ``!``."""
        used, degraded = [], False
        for field, label in FUSION_INPUTS:
            value = int(getattr(m, field))
            if value == 1:
                used.append(label)
            elif value == 2:
                used.append(f"{label}!")
                degraded = True
        if not used:
            return "fuse nothing", RED
        return "fuse " + " ".join(used), AMBER if degraded else GREEN

    @staticmethod
    def _waiting(m: FpaOdomstatus) -> str:
        """What the fusion still waits for, IMU bias first, else the wheelspeed."""
        if int(m.imu_status) != 3:
            return IMU_CONV.get(int(m.imu_conv), "")
        if int(m.ws_status) in (2, 3):  # enabled and converging
            return WS_CONV.get(int(m.ws_conv), "")
        return ""

    # ------------------------------------------------------------------ panel
    def _lines(self) -> list[str]:
        m = self._msg
        age = self._age()
        self._stale = age is not None and age > self.stale_timeout
        if m is None or age is None:
            return [
                self._line([("FP ", GREY), ("no data", GREY)]),
                self._line([(self.topic or "(no topic)", GREY)]),
            ]

        if self._stale:
            # Stale statuses mislead: say so on the headline, and _line greys the rest.
            head = [("FP ", GREY), (f"silent {age:.0f} s", RED)]
        else:
            init, color = self._lookup(INIT_STATUS, m.init_status)
            head = [("FP ", GREY), (init, color), (SEP, GREY), self._fusion(m)]
            wait = self._waiting(m)
            if wait:
                head += [(SEP, GREY), (f"wait {wait}", AMBER)]

        gnss1, c1 = self._lookup(GNSS_STATUS, m.gnss1_status)
        gnss2, c2 = self._lookup(GNSS_STATUS, m.gnss2_status)
        corr, cc = self._lookup(CORR_STATUS, m.corr_status)
        imu, ci = self._lookup(IMU_STATUS, m.imu_status)
        ws, cw = self._lookup(WS_STATUS, m.ws_status)
        detail = [
            ("g1 ", GREY),
            (gnss1, c1),
            (SEP + "g2 ", GREY),
            (gnss2, c2),
            (SEP + "cor ", GREY),
            (corr, cc),
            (SEP + "imu ", GREY),
            (imu, ci),
            (SEP + "ws ", GREY),
            (ws, cw),
        ]
        return [self._line(head, stale_grey=False), self._line(detail)]

    def _panel(self) -> OverlayText:
        lines = self._lines()
        # Red panel while the fusion is not initialised at all or the sensor went silent,
        # so it reads across the room like the e-stop does on the mission status panel.
        age = self._age()
        bad = (
            age is None
            or age > self.stale_timeout
            or int(self._msg.init_status) not in (1, 2)
        )
        msg = OverlayText()
        msg.action = OverlayText.ADD
        msg.width = self.panel_width
        # Grow the box with the content instead of leaving a large empty rectangle.
        msg.height = int(round(len(lines) * self.text_size * 1.6 + 14))
        msg.horizontal_alignment = (
            OverlayText.RIGHT if self.align_right else OverlayText.LEFT
        )
        msg.vertical_alignment = (
            OverlayText.BOTTOM if self.align_bottom else OverlayText.TOP
        )
        msg.horizontal_distance = self.margin
        msg.vertical_distance = self.margin
        msg.bg_color = (
            rgba(0.35, 0.02, 0.02, 0.75)
            if bad
            else rgba(0.06, 0.06, 0.08, self.bg_alpha)
        )
        msg.fg_color = rgba(0.91, 0.92, 0.93, 1.0)
        msg.line_width = 2
        msg.text_size = self.text_size
        msg.font = self.font
        msg.text = "<br>".join(lines) if self.markup else "\n".join(lines)
        return msg

    def _tick(self) -> None:
        self.pub.publish(self._panel())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixpositionHud()
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
