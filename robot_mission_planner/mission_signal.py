#!/usr/bin/env python3
"""
mission_signal: turn ``road_follower`` mission events into something a bystander notices.

Robotour requires the robot to indicate that it has arrived at the goal, and homologation
tests "signalization, manual load, QR-code entry and continue of trial" (review R1). The
follower publishes every mission step on ``~/event`` as a latched ``std_msgs/String``
(``GOAL:lat,lon``, ``PLANNING``, ``ROUTE:...``, ``START``, ``ARRIVED``, ``CONTINUE``,
``ABORT:<state>``, ``PLAN_FAILED:...``, ``IDLE``); this node maps the event *name* (the part
before the first ``:``) to an action from a table in ``config/mission_signal.yaml``.

Backends (``backend``)
----------------------
speak : (default) publish the sentence for ``helhest_bringup``'s ``nodes/speak.py`` ->
        sound_play, on the topic of the event's level (``speech_level.<EVENT>``: ``info`` ->
        ``speak_info_topic``, ``warn`` -> ``speak_warn_topic``, ``error`` ->
        ``speak_error_topic``; unlisted events are ``info``). speak.py plays the levels at
        rising volume. Needs the NUC's ``sound.launch``; with nothing subscribed the events
        are still logged.
aplay : play a wav with ``aplay`` in a child process, so the node never blocks on audio.
        Table entries are absolute paths or file names under ``sound_dir``; a missing file
        is warned about once and then ignored, rather than taking the node with it.
log   : only log the event. Useful in a replay or on a robot without a speaker.
gpio  : placeholder for the light / GPIO backend, see ``_signal_gpio``.

The table is per backend -- ``sounds.<EVENT>`` for ``aplay``, ``speech.<EVENT>`` for
``speak`` -- so switching the backend does not mean rewriting the config.

Latched events
--------------
A node started mid-mission immediately receives the *last* event of the run, which is usually
not news (nobody wants the arrival sound when the signal node is restarted after the
arrival), and ``std_msgs/String`` carries no stamp to measure its age by. The first message
received within ``ignore_latched_s`` of the node start is therefore dropped -- including a
genuinely new event in that first second, and not including a latched one that arrives later
through slow discovery.
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

# Default table: every follower event worth a sound. The files need not exist (a missing one
# degrades to a single warning), so the table can be filled in as the wavs are recorded.
# An empty value switches an event off.
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

# Which speak.py topic an event is said on; speak.py sets the volume by it (info 0.6, warn 0.8,
# error 1.0). An event not listed here is said at info.
DEFAULT_SPEECH_LEVELS = {
    "ABORT": "warn",
    "PLAN_FAILED": "error",
}

SPEECH_LEVELS = ("info", "warn", "error")

BACKENDS = ("speak", "aplay", "log", "gpio")


class MissionSignal(Node):
    def __init__(self) -> None:
        # The table lives in the yaml as speech.<EVENT> / sounds.<EVENT>, so it cannot be
        # declared up front: declaring from the overrides instead lets an event be added
        # without touching this node.
        super().__init__(
            "mission_signal", automatically_declare_parameters_from_overrides=True
        )

        event_topic = self._param("event_topic", "/road_follower/event")
        self.backend = str(
            self._param("backend", "speak")
        )  # speak | aplay | log | gpio
        # speak backend: helhest_bringup nodes/speak.py listens on one topic per level
        # (sensor_data QoS); "" switches a level off, its events are then only logged.
        speak_topics = {
            "info": str(self._param("speak_info_topic", "/speak/info")),
            "warn": str(self._param("speak_warn_topic", "/speak/warn")),
            "error": str(self._param("speak_error_topic", "/speak/err")),
        }
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
        self.levels = dict(DEFAULT_SPEECH_LEVELS)
        for name, param in self.get_parameters_by_prefix("speech_level").items():
            level = str(param.value).strip().lower()
            level = "error" if level == "err" else level  # the topic's own spelling
            if level not in SPEECH_LEVELS:
                self.get_logger().error(
                    f"speech_level.{name}: unknown level '{param.value}' "
                    f"(expected one of {SPEECH_LEVELS}), using 'info'"
                )
                level = "info"
            self.levels[name.upper()] = level

        self._speak_pubs = (
            {
                level: self.create_publisher(String, topic, qos_profile_sensor_data)
                for level, topic in speak_topics.items()
                if topic
            }
            if self.backend == "speak"
            else {}
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
                " on " + ", ".join(f"{lvl}={t}" for lvl, t in speak_topics.items() if t)
                if self.backend == "speak"
                else f", sounds in {self.sound_dir}"
            )
            + ", table: "
            + ", ".join(
                f"{k}={v}"
                + (f" [{self.levels.get(k, 'info')}]" if self.backend == "speak" else "")
                for k, v in sorted(self.table.items())
                if v
            )
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
        except Exception as exc:  # noqa: BLE001 - not built / not sourced: use the source tree
            self.get_logger().warning(
                f"package share not found ({exc!r}); sounds from the source tree"
            )
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
        level = self.levels.get(event, "info")
        pub = self._speak_pubs.get(level)
        if pub is None:
            self._warn_once(
                f"speak:{level}",
                f"The {level} speech topic is off: {level} events are only logged",
            )
            return
        pub.publish(String(data=action))

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
        name (``ARRIVED``, ``CONTINUE``, ...) and ``action`` its table entry, free-form for
        this backend (``"blink:3"``, a GPIO line name) since it never reaches the file
        system. Everything else already works; only this method has to be filled in.
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
