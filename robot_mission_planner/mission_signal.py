#!/usr/bin/env python3
"""
mission_signal: turn ``road_follower`` mission events into something a bystander notices.

Robotour requires the robot to indicate that it has arrived at the goal, and the
homologation tests "signalization, manual load, QR-code entry and continue of trial"
(review R1). The follower already publishes every mission step on ``~/event`` as a latched
``std_msgs/String`` (``GOAL:lat,lon``, ``PLANNING``, ``ROUTE:...``, ``START``, ``ARRIVED``,
``CONTINUE``, ``ABORT:<state>``, ``PLAN_FAILED:...``, ``IDLE``); this node maps the event
*name* (the part before the first ``:``) to an action from a table in
``config/mission_signal.yaml`` and runs it.

Backends (``backend``)
----------------------
speak : (default) say it. ``helhest_bringup``'s ``nodes/speak.py`` subscribes
        ``std_msgs/String`` on ``/speak/info`` | ``/speak/warn`` | ``/speak/err`` and hands
        the text to sound_play, so the table holds sentences ("Arrived") and this node only
        publishes on ``speak_topic``. Needs the NUC's ``sound.launch`` (speaker) to be up;
        with nothing subscribed the events are still logged.
aplay : play a wav file with ``aplay`` in a child process, so the node never blocks on
        audio. Table entries are file names relative to the package ``data/`` directory
        (``sound_dir``) or absolute paths. A missing file is warned about once and then
        ignored - an unplugged speaker or a wav that was never copied to the robot must
        not take the mission node with it.
log   : only log the event. Useful in a replay or on a robot without a speaker.
gpio  : placeholder for the light / GPIO backend, see ``_signal_gpio``.

The table is per backend: ``sounds.<EVENT>`` (wav files) for ``aplay``, ``speech.<EVENT>``
(sentences) for ``speak``, so switching the backend does not mean rewriting the config.

Latched events
--------------
The follower's event topic is latched, so a node started mid-mission immediately receives
the *last* event of the run, which is usually not news (nobody wants the arrival sound when
the signal node is restarted after the arrival). ``std_msgs/String`` carries no stamp, so
the age of that message cannot be measured: the first message received within
``ignore_latched_s`` of the node start is dropped instead. Limitation: a genuinely new
event in that first second is dropped with it, and a latched event that arrives later
(a slow discovery) is played.
"""

from __future__ import annotations

import os
import shlex
import subprocess

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String

# Default table: every follower event that is worth a sound. The files do not have to
# exist - a missing one degrades to a single warning - so the table can be filled in as
# the wavs are recorded. An empty value switches an event off.
DEFAULT_SOUNDS = {
    "GOAL": "goal.wav",
    "ROUTE": "route.wav",
    "START": "start.wav",
    "ARRIVED": "arrived.wav",
    "ABORT": "abort.wav",
    "PLAN_FAILED": "plan_failed.wav",
    "IDLE": "idle.wav",
}

# What the speak backend says for the same events (helhest_bringup speak.py -> sound_play).
DEFAULT_SPEECH = {
    "GOAL": "Goal accepted",
    "ROUTE": "Route planned",
    "START": "Starting",
    "ARRIVED": "Arrived",
    "ABORT": "Aborted",
    "PLAN_FAILED": "Planning failed",
    "IDLE": "Waiting for a goal",
}

BACKENDS = ("speak", "aplay", "log", "gpio")


class MissionSignal(Node):
    def __init__(self) -> None:
        # The table lives in the yaml as speech.<EVENT> / sounds.<EVENT>, so the parameters
        # cannot all be declared up front: whatever the config file carries is declared from
        # the overrides, which also lets an event be added without touching this node.
        super().__init__(
            "mission_signal", automatically_declare_parameters_from_overrides=True
        )

        event_topic = self._param("event_topic", "/road_follower/event")
        self.backend = str(
            self._param("backend", "speak")
        )  # speak | aplay | log | gpio
        # speak backend: helhest_bringup nodes/speak.py listens here (sensor_data QoS).
        speak_topic = str(self._param("speak_topic", "/speak/info"))
        # Command the aplay backend runs; the resolved file path is appended to it.
        self.player_command = str(self._param("player_command", "aplay -q"))
        # Where relative sound files are looked for (empty = the package data/ directory).
        self.sound_dir = str(self._param("sound_dir", "")) or self._default_sound_dir()
        # See "Latched events" above: the first message within this many seconds of the
        # start is treated as the latched backlog of a previous run and dropped (0 = off).
        self.ignore_latched_s = float(self._param("ignore_latched_s", 1.0))

        if self.backend not in BACKENDS:
            self.get_logger().error(f"Unknown backend '{self.backend}', using 'log'")
            self.backend = "log"
        # One table per backend: file names for aplay, sentences for speak (and for the log
        # backend, which is what an operator wants to read).
        prefix = "sounds" if self.backend == "aplay" else "speech"
        self.table = dict(DEFAULT_SOUNDS if self.backend == "aplay" else DEFAULT_SPEECH)
        for name, param in self.get_parameters_by_prefix(prefix).items():
            self.table[name.upper()] = "" if param.value is None else str(param.value)

        self._speak_pub = (
            self.create_publisher(String, speak_topic, qos_profile_sensor_data)
            if self.backend == "speak"
            else None
        )

        self._warned: set[str] = set()  # files / commands already complained about
        self._children: list[subprocess.Popen] = []
        self._first_message = True
        self._start = self.get_clock().now().nanoseconds * 1e-9

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, event_topic, self._event_callback, latched)

        self.get_logger().info(
            f"mission_signal ready: {event_topic} -> {self.backend} backend"
            + (
                f" on {speak_topic}"
                if self.backend == "speak"
                else f", sounds in {self.sound_dir}"
            )
            + ", table: "
            + ", ".join(f"{k}={v}" for k, v in sorted(self.table.items()) if v)
        )

    # ------------------------------------------------------------------ helpers
    def _param(self, name: str, default):
        """Declare (unless the yaml already brought it in) and read one parameter."""
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        value = self.get_parameter(name).value
        return default if value is None else value

    def _default_sound_dir(self) -> str:
        try:
            return os.path.join(
                get_package_share_directory("robot_mission_planner"), "data"
            )
        except Exception:  # noqa: BLE001 - not built / not sourced: use the source tree
            return os.path.join(os.path.dirname(__file__), "..", "data")

    def _warn_once(self, key: str, message: str) -> None:
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warning(message)

    def _reap(self) -> None:
        self._children = [c for c in self._children if c.poll() is None]

    # ------------------------------------------------------------------ input
    def _event_callback(self, msg: String) -> None:
        text = msg.data.strip()
        if self._first_message:
            self._first_message = False
            age = self.get_clock().now().nanoseconds * 1e-9 - self._start
            if self.ignore_latched_s > 0 and age < self.ignore_latched_s:
                self.get_logger().info(
                    f"Ignoring the latched event {text!r} from before the start"
                )
                return
        name = text.split(":", 1)[0].strip().upper()
        action = self.table.get(name)
        if not action:
            self.get_logger().info(f"Event {text!r}: nothing to signal")
            return
        self.get_logger().info(f"Event {text!r} -> {self.backend} {action}")
        if self.backend == "speak":
            self._signal_speak(name, action)
        elif self.backend == "aplay":
            self._signal_aplay(name, action)
        elif self.backend == "gpio":
            self._signal_gpio(name, action)

    # ------------------------------------------------------------------ backends
    def _signal_speak(self, event: str, action: str) -> None:
        """Hand the sentence to helhest_bringup's speak.py (which talks to sound_play)."""
        self._speak_pub.publish(String(data=action))

    def _signal_aplay(self, event: str, action: str) -> None:
        """Play a wav in a child process; nothing here may block or raise."""
        path = action if os.path.isabs(action) else os.path.join(self.sound_dir, action)
        if not os.path.exists(path):
            self._warn_once(
                path,
                f"No sound file for {event}: {path} (event only logged from now on)",
            )
            return
        self._reap()
        try:
            self._children.append(
                subprocess.Popen(  # noqa: S603 - the command comes from our own parameters
                    shlex.split(self.player_command) + [path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            )
        except Exception as e:  # noqa: BLE001 - no player, no sound card, ...
            self._warn_once(
                self.player_command, f"Cannot run '{self.player_command}': {e}"
            )

    def _signal_gpio(self, event: str, action: str) -> None:
        """
        HOOK: light / GPIO backend (R1), not implemented yet.

        Drive the signal lamp from here once it is wired: ``event`` is the follower event
        name (``ARRIVED``, ``CONTINUE``, ...) and ``action`` its table entry, which for this
        backend is free-form (``"blink:3"``, a GPIO line name, ...) because it never reaches
        the file system. Everything else - the table, the topic, the latched-event guard -
        already works, so only this method has to be filled in.
        """
        self._warn_once(
            "gpio",
            f"backend 'gpio' is a placeholder: {event} -> {action} not signalled",
        )

    def destroy_node(self) -> bool:
        for child in self._children:
            if child.poll() is None:
                child.terminate()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionSignal()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
