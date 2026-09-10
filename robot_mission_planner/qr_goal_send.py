#!/usr/bin/env python3
"""
qr_goal_send: enter a Robotour goal by hand.

    qr_goal_send "geo:50.1103476,14.4159857"      # through the qr_goal node (~/text)
    qr_goal_send 50.1103476,14.4159857 --direct   # straight to /qr_goal/goal (no qr_goal node)
    qr_goal_send --home                           # back to the service area (see --home-file)

The competition gives the team the loading-zone QR in the service area, so the
payload can be typed in instead of shown to the camera. Same parser as the node.
``--home`` sends the coordinate road_follower recorded at the first goal of the
run (its own fix in the service area), so the return leg needs no typing at all.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

from robot_mission_planner.qr_goal import parse_geo_uri

# Written by road_follower when it accepts the first goal of a run (parameter mission_dir).
DEFAULT_HOME_FILE = "~/missions/home.txt"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Publish a Robotour goal (geo:lat,lon or lat,lon)."
    )
    ap.add_argument(
        "payload", nargs="?", help="geo:lat,lon (as printed in the QR code) or lat,lon"
    )
    ap.add_argument(
        "--home",
        action="store_true",
        help="send the home coordinate road_follower recorded at the first goal of the run",
    )
    ap.add_argument(
        "--home-file",
        default=DEFAULT_HOME_FILE,
        help=f"file --home reads (default {DEFAULT_HOME_FILE})",
    )
    ap.add_argument(
        "--text-topic", default="/qr_goal/text", help="qr_goal node text input"
    )
    ap.add_argument(
        "--direct",
        action="store_true",
        help="publish GeoPointStamped on --goal-topic instead",
    )
    ap.add_argument("--goal-topic", default="/qr_goal/goal")
    ap.add_argument("--frame-id", default="wgs84")
    ap.add_argument(
        "--wait",
        type=float,
        default=5.0,
        help="s to wait for a subscriber before publishing",
    )
    ap.add_argument(
        "--hold",
        type=float,
        default=2.0,
        help="s to keep the latched publisher alive afterwards, for subscribers that "
        "match late (any subscriber ends --wait, not necessarily road_follower)",
    )
    args = ap.parse_args(argv)

    if args.home:
        path = os.path.expanduser(args.home_file)
        try:
            with open(path) as f:
                payload = f.read().strip()
        except OSError as e:
            print(f"cannot read the home file {path}: {e}", file=sys.stderr)
            return 2
        print(f"home from {path}: {payload}")
    elif args.payload:
        payload = args.payload
    else:
        ap.error("give a payload or --home")

    latlon = parse_geo_uri(payload)
    if latlon is None:
        print(f"not a geo position: {payload!r}", file=sys.stderr)
        return 2

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile

    rclpy.init()
    node = Node("qr_goal_send")
    latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    try:
        if args.direct:
            from geographic_msgs.msg import GeoPointStamped

            pub = node.create_publisher(GeoPointStamped, args.goal_topic, latched)
            msg = GeoPointStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = args.frame_id
            msg.position.latitude, msg.position.longitude = latlon
            where = args.goal_topic
        else:
            from std_msgs.msg import String

            pub = node.create_publisher(String, args.text_topic, latched)
            msg = String(data=payload)
            where = args.text_topic
        # Like `ros2 topic pub`: wait for the receiver to match before publishing, so the
        # message is not lost in DDS discovery; the publisher is latched either way.
        t0 = time.monotonic()
        while pub.get_subscription_count() == 0 and time.monotonic() - t0 < args.wait:
            rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() == 0:
            print(
                f"warning: nobody subscribed to {where} within {args.wait:g} s",
                file=sys.stderr,
            )
        pub.publish(msg)
        print(f"published {latlon[0]:.7f}, {latlon[1]:.7f} on {where}")
        # The sample is only retained while this process lives, so hold the node open:
        # --wait ends at the *first* subscriber, which may not be the one that matters.
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.hold:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
