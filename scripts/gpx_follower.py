#!/usr/bin/env python3

import os
import argparse

import utm
import yaml
import gpxpy
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints, FollowGPSWaypoints, NavigateThroughPoses


class GPXFollower(Node):
    def __init__(self, gps_file, args):
        super().__init__("gps_follower")
        self.args = args

        if self.args.navigate_through_poses:
            self._action_client = ActionClient(
                self, NavigateThroughPoses, "navigate_through_poses"
            )
        else:
            if not args.use_utm:
                self._action_client = ActionClient(
                    self, FollowGPSWaypoints, "follow_gps_waypoints"
                )
            else:
                self._action_client = ActionClient(
                    self, FollowWaypoints, "follow_waypoints"
                )

        self.gps_file = gps_file
        if not os.path.exists(self.gps_file):
            self.get_logger().error(f"GPS file {self.gps_file} does not exist")
            exit(1)

        if self.gps_file.endswith(".gpx"):
            self.parse_gpx_file()
        elif self.gps_file.endswith(".yaml"):
            self.parse_yaml_file()
        else:
            self.get_logger().error(
                "Unsupported file format. Please provide a .gpx or .yaml file."
            )
            exit(1)

        # Wait for the action server to be available
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            if self.args.navigate_through_poses:
                self.get_logger().info(
                    f"Waiting for /navigate_through_poses action server..."
                )
            else:
                self.get_logger().info(
                    f"Waiting for {
                        '/follow_gps_waypoints'
                        if not self.args.use_utm
                        else '/follow_waypoints'
                    } action server..."
                )
        self.goal_handle = None

        self.get_logger().info("GPXFollower node initialized.")

    def _convert_waypoint(self, waypoint):
        if not self.args.use_utm:
            pose = GeoPose()
            pose.position.latitude = waypoint["lat"]
            pose.position.longitude = waypoint["lon"]
            pose.position.altitude = waypoint["ele"] if waypoint["ele"] else 0.0
        else:
            pose = PoseStamped()
            pose.header.frame_id = "utm"
            pose.header.stamp = self.get_clock().now().to_msg()
            utm_coords = utm.from_latlon(waypoint["lat"], waypoint["lon"])
            pose.pose.position.x = utm_coords[0]
            pose.pose.position.y = utm_coords[1]
            pose.pose.position.z = waypoint["ele"] if waypoint["ele"] else 0.0

        return pose

    def parse_gpx_file(self):
        waypoints = []
        try:
            with open(self.gps_file, "r") as file:
                gpx = gpxpy.parse(file)
            for waypoint in gpx.waypoints:
                point = {
                    "lat": waypoint.latitude,
                    "lon": waypoint.longitude,
                    "ele": waypoint.elevation or None,
                }
                waypoints.append(self._convert_waypoint(point))
        except Exception as e:
            self.get_logger().error(f"Error parsing GPX file: {e}")
            return []
        self.waypoints = waypoints
        if not self.waypoints:
            self.get_logger().error("No waypoints found in GPX file.")
        else:
            self.get_logger().info(f"Parsed {len(waypoints)} waypoints from GPX file.")

    def parse_yaml_file(self):
        waypoints = []
        with open(self.gps_file, "r") as f:
            file_waypoints = yaml.safe_load(f)["waypoints"]
        for waypoint in file_waypoints:
            point = {"lat": waypoint["latitude"], "lon": waypoint["longitude"]}
            if "elevation" in waypoint:
                point["ele"] = waypoint["elevation"]
            else:
                point["ele"] = None
            waypoints.append(self._convert_waypoint(point))
        self.waypoints = waypoints
        if not self.waypoints:
            self.get_logger().error("No waypoints found in YAML file.")
        else:
            self.get_logger().info(f"Parsed {len(waypoints)} waypoints from YAML file.")

    def send_path(self):
        if not self.waypoints:
            return

        if self.args.navigate_through_poses:
            waypoint_msg = NavigateThroughPoses.Goal()
            waypoint_msg.poses = self.waypoints
            waypoint_msg.behavior_tree = "MainTree"         # TODO: could be wrong, taken from VP config   
        else:
            if not self.args.use_utm:
                waypoint_msg = FollowGPSWaypoints.Goal()
                waypoint_msg.gps_poses = self.waypoints
            else:
                waypoint_msg = FollowWaypoints.Goal()
                waypoint_msg.poses = self.waypoints

        self.get_logger().info(f"Sending {len(self.waypoints)} waypoints to follow")
        send_goal_future = self._action_client.send_goal_async(
            waypoint_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error("Goal rejected by server")
            return

        self.get_logger().info("Goal accepted by server")
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Current waypoint index: {feedback.current_waypoint}")

    def result_callback(self, future):
        result = future.result().result
        if result.missed_waypoints:
            self.get_logger().warn(f"Missed waypoints: {result.missed_waypoints}")
        else:
            self.get_logger().info("Successfully navigated all waypoints")
        rclpy.shutdown()

    def cancel_goal(self):
        if self.goal_handle is not None:
            self.get_logger().info("Cancelling the current goal")
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_response_callback)
        else:
            self.get_logger().warn("No goal to cancel")

    def cancel_response_callback(self, future):
        cancel_result = future.result()
        if cancel_result.goals_canceling:
            self.get_logger().info("Goal successfully canceled")
        else:
            self.get_logger().warn("Failed to cancel goal")


def parse_args():
    parser = argparse.ArgumentParser(description="GPX Follower Node")

    parser.add_argument(
        "-f",
        "--file",
        type=str,
        required=True,
        help="Path to the GPX of YAML file containing waypoints",
    )
    parser.add_argument(
        "--use-utm",
        action="store_true",
        help="Convert waypoints to UTM coordinates",
    )
    parser.add_argument(
        "--navigate-through-poses",
        action="store_true",
        help="Use action NavigateThroughPoses instead of FollowWaypoints",
    )

    return parser.parse_args()


def main(args):
    gpx_file_path = args.file
    gpx_file_path = os.path.join(os.path.dirname(__file__), "../", gpx_file_path)
    if not os.path.exists(gpx_file_path):
        print(f"GPX file {gpx_file_path} does not exist")
        return

    rclpy.init()
    node = GPXFollower(gpx_file_path, args)
    try:
        node.send_path()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        node.cancel_goal()
        node.destroy_node()


if __name__ == "__main__":
    args = parse_args()
    main(args)
