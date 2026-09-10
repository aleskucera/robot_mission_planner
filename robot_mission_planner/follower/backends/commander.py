"""crl_commander on the Helhest NUC: goto for one pose, sequence for a waypoint list."""

from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String

from robot_mission_planner.follower.backends.base import Backend
from robot_mission_planner.follower.frames import latlon_to_ecef

import math


class CommanderBackend(Backend):
    """
    Goals are published, modes are switched over services, progress is inferred.

    Waypoints go out in ECEF (``earth_frame``): the commander transforms them into its own
    map frame itself, so a sequence stays valid when the local ENU origin is redefined
    between runs. Road goals go out as a single pose in ``map_frame`` (``goto``).
    """

    kind = "commander"

    def __init__(
        self,
        node,
        frames,
        *,
        earth_frame: str,
        goal_waypoint_topic: str,
        goal_sequence_topic: str,
        switch_mode_service: str,
        configure_sequence_service: str,
        state_topic: str,
        restart_gap: float = 5.0,
        service_timeout: float = 3.0,
        mode_settle_time: float = 2.0,
    ):
        super().__init__(node, frames)
        from crl_commander.srv import ConfigureSequenceMode, SwitchMode

        self.earth_frame = earth_frame
        self.restart_gap = float(restart_gap)
        self.service_timeout = float(service_timeout)
        self.mode_settle_time = float(mode_settle_time)
        self._srv_types = {"switch": SwitchMode, "configure": ConfigureSequenceMode}

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_goal = node.create_publisher(PoseStamped, goal_waypoint_topic, latched)
        self._pub_sequence = node.create_publisher(PoseArray, goal_sequence_topic, latched)
        self._cli_switch = node.create_client(SwitchMode, switch_mode_service)
        self._cli_configure = node.create_client(ConfigureSequenceMode, configure_sequence_service)
        node.create_subscription(String, state_topic, self._state_callback, 10)

        self.mode = None  # what the commander last reported
        self._requested_mode = None  # what we last asked for (the state topic lags)
        self._requested_time = 0.0
        self._state_time = None
        self._restarted = False
        self._watchdogs = []

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    # ---------------------------------------------------------------- waypoints
    def to_src(self, point):
        return latlon_to_ecef(point["lat"], point["lon"], point.get("ele", 0.0))

    def waypoint_msg(self, point):
        msg = PoseStamped()
        msg.header.frame_id = self.earth_frame
        x, y, z = self.to_src(point)
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        msg.pose.orientation.w = 1.0
        return msg

    # ---------------------------------------------------------------- goals
    def send_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.header.frame_id = self.frames.map_frame
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self._pub_goal.publish(pose)
        self.switch_mode("goto")

    def send_sequence(self, waypoints, loop: bool = False):
        seq = PoseArray()
        seq.header.frame_id = waypoints[0].header.frame_id
        seq.header.stamp = self.node.get_clock().now().to_msg()
        seq.poses = [wp.pose for wp in waypoints]
        self.configure_sequence(loop, lambda: self._publish_sequence(seq))

    def cancel(self):
        self.switch_mode("stop")

    # ---------------------------------------------------------------- services
    def _publish_sequence(self, seq: PoseArray):
        self._pub_sequence.publish(seq)
        self.switch_mode("sequence")

    def configure_sequence(self, loop: bool, then):
        """
        Make the commander take its sequence from the topic, then call ``then``.

        Sent before every sequence, not once: a restarted commander is back at its launch
        default (2026-09-08 it loaded a GPX file from disk mid-mission), and the call is cheap.
        """
        cli = self._cli_configure
        if not cli.service_is_ready():
            self.log.warning(
                f"{cli.srv_name} not ready; publishing the sequence anyway (commander must be "
                "configured with sequence_source=topic)."
            )
            then()
            return
        req = self._srv_types["configure"].Request()
        req.source = req.SOURCE_TOPIC
        req.gpx_file_name = ""
        req.loop = bool(loop)
        called = {"then": False}

        def run_then():
            if not called["then"]:
                called["then"] = True
                then()

        def done(fut):
            try:
                res = fut.result()
                self.log.info(f"configure_sequence_mode: {res.success} {res.message}")
            except Exception as e:
                self.log.error(f"configure_sequence_mode failed: {e}")
            run_then()

        future = cli.call_async(req)
        future.add_done_callback(done)
        self.watch_service_call(future, "configure_sequence_mode", on_timeout=run_then)

    def switch_mode(self, mode: str):
        # The state topic lags the request; remember what we asked for so that a burst of
        # path messages does not turn into a burst of identical service calls.
        if self._requested_mode == mode:
            return
        if self.mode is not None and self.mode.lower() == mode:
            self._requested_mode = mode
            self._requested_time = self._now()
            return
        cli = self._cli_switch
        if not cli.service_is_ready():
            self.log.warning(f"{cli.srv_name} not ready; cannot switch to '{mode}'.")
            return
        self._requested_mode = mode
        self._requested_time = self._now()
        req = self._srv_types["switch"].Request()
        req.mode = mode

        def reset_request():
            if self._requested_mode == mode:
                self._requested_mode = None  # allow a retry

        def done(fut):
            try:
                res = fut.result()
                level = self.log.info if res.success else self.log.error
                level(f"switch_mode('{mode}'): {res.success} {res.message}")
                if not res.success:
                    reset_request()
            except Exception as e:
                self.log.error(f"switch_mode('{mode}') failed: {e}")
                reset_request()

        future = cli.call_async(req)
        future.add_done_callback(done)
        self.watch_service_call(future, f"switch_mode('{mode}')", on_timeout=reset_request)

    def watch_service_call(self, future, what, on_timeout=None):
        """Log (and optionally react) when a service call does not return in time."""
        if self.service_timeout <= 0:
            return

        def check():
            timer.cancel()
            self._watchdogs = [t for t in self._watchdogs if t is not timer]
            if not future.done():
                self.log.error(f"{what} did not respond within {self.service_timeout} s")
                if on_timeout:
                    on_timeout()

        timer = self.node.create_timer(self.service_timeout, check)
        self._watchdogs.append(timer)

    # ---------------------------------------------------------------- status
    def _state_callback(self, msg):
        if msg.data != self.mode:
            self.log.info(f"Commander state: {msg.data}")
        now = self._now()
        if (
            self.restart_gap > 0
            and self._state_time is not None
            and (now - self._state_time) > self.restart_gap
        ):
            self.log.warning(
                f"Commander state silent for {now - self._state_time:.0f} s: "
                "assuming a restart, re-sending the current goal."
            )
            self._restarted = True
            self._requested_mode = None
        self._state_time = now
        self.mode = msg.data
        # The commander leaves our mode on its own (sequence finished -> STOP, operator
        # intervention, STUCK). Forget the request so the next switch is actually sent.
        if (
            self._requested_mode is not None
            and msg.data.lower() != self._requested_mode
            and (now - self._requested_time) > self.mode_settle_time
        ):
            self._requested_mode = None

    @property
    def state_text(self):
        return self.mode

    def stuck(self) -> bool:
        return self.mode is not None and "STUCK" in self.mode.upper()

    def take_restarted(self) -> bool:
        restarted, self._restarted = self._restarted, False
        return restarted

    def left_us(self, goal_active: bool) -> bool:
        """True once the commander sits in STOP although we asked for goto/sequence."""
        return (
            self.mode is not None
            and self.mode.upper() == "STOP"
            and self._requested_mode is None
            and goal_active
        )
