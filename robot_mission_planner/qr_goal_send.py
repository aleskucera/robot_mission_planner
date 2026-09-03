#!/usr/bin/env python3
"""
qr_goal_send: enter a Robotour goal by hand.

    qr_goal_send "geo:50.1103476,14.4159857"      # through the qr_goal node (~/text)
    qr_goal_send 50.1103476,14.4159857 --direct   # straight to /route_planner/goal

The competition gives the team the loading-zone QR in the service area, so the
payload can be typed in instead of shown to the camera. Same parser as the node.
"""

from __future__ import annotations

import argparse
import sys
import time

from robot_mission_planner.qr_goal import parse_geo_uri


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Publish a Robotour goal (geo:lat,lon or lat,lon).")
    ap.add_argument("payload", help="geo:lat,lon (as printed in the QR code) or lat,lon")
    ap.add_argument("--text-topic", default="/qr_goal/text", help="qr_goal node text input")
    ap.add_argument(
        "--direct", action="store_true", help="publish GeoPointStamped on --goal-topic instead"
    )
    ap.add_argument("--goal-topic", default="/route_planner/goal")
    ap.add_argument("--frame-id", default="wgs84")
    ap.add_argument("--wait", type=float, default=5.0, help="s to wait for a subscriber before publishing")
    args = ap.parse_args(argv)

    latlon = parse_geo_uri(args.payload)
    if latlon is None:
        print(f"not a geo position: {args.payload!r}", file=sys.stderr)
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
            msg = String(data=args.payload)
            where = args.text_topic
        # Like `ros2 topic pub`: wait for the receiver to match before publishing, so the
        # message is not lost in DDS discovery; the publisher is latched either way.
        t0 = time.monotonic()
        while pub.get_subscription_count() == 0 and time.monotonic() - t0 < args.wait:
            rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() == 0:
            print(f"warning: nobody subscribed to {where} within {args.wait:g} s", file=sys.stderr)
        pub.publish(msg)
        print(f"published {latlon[0]:.7f}, {latlon[1]:.7f} on {where}")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.5:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
