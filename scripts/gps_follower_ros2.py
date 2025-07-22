#!/usr/bin/env python3

import os
import time
import argparse

import utm
import yaml
import json
import gpxpy
import requests

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints, FollowGPSWaypoints, NavigateThroughPoses

# Define the server URL
SERVER_URL = "http://45.91.169.180:5001/api/update_data"


class GPXFollower(Node):
    def __init__(self, gps_file, args):
        super().__init__("gps_follower")
        self.args = args
        self.data = {"robot_id": args.robot}

        if self.args.navigate_through_poses:
            self._action_client = ActionClient(
                self, NavigateThroughPoses, "navigate_through_poses"
            )
            self.args.use_utm = True
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
        elif self.gps_file.endswith((".yaml", ".yml")):
            self.parse_yaml_file()
        else:
            self.get_logger().error(
                "Unsupported file format. Please provide a .gpx or .yaml file."
            )
            exit(1)

        if self.args.reverse:
            self.waypoints.reverse()
            self.waypoints_gps.reverse()
        self.waypoints = self.waypoints[self.args.start :]
        self.waypoints_gps = self.waypoints_gps[self.args.start :]

        self.number_waypoints = len(self.waypoints)
        self.current_waypoint = 0
        self.pose_gps = None
        self.pose_ekf = None
        self.path_send_url = False
        self.get_logger().info(
            f"Starting from waypoint index {self.args.start}, total waypoints: {self.number_waypoints}"
        )

        # Wait for the action server to be available
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            if self.args.navigate_through_poses:
                self.get_logger().info(
                    "Waiting for /navigate_through_poses action server..."
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

        self.sub_gps = self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_callback, 10
        )
        self.sub_ekf = self.create_subscription(
            NavSatFix, "/gps/filtered", self.ekf_callback, 10
        )

        self.get_logger().info("GPSFollower node initialized.")

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
        waypoints_gps = []
        try:
            with open(self.gps_file, "r") as file:
                gpx = gpxpy.parse(file)
            for waypoint in gpx.waypoints:
                point = {
                    "lat": waypoint.latitude,
                    "lon": waypoint.longitude,
                }
                waypoints_gps.append(point)
                point["ele"] = waypoint.elevation or None
                waypoints.append(self._convert_waypoint(point))
        except Exception as e:
            self.get_logger().error(f"Error parsing GPX file: {e}")
            return []
        self.waypoints = waypoints
        self.waypoints_gps = waypoints_gps

        if not self.waypoints:
            self.get_logger().error("No waypoints found in GPX file.")
        else:
            self.get_logger().info(f"Parsed {len(waypoints)} waypoints from GPX file.")

    def parse_yaml_file(self):
        waypoints = []
        waypoints_gps = []
        with open(self.gps_file, "r") as f:
            file_waypoints = yaml.safe_load(f)["waypoints"]
        for waypoint in file_waypoints:
            point = {"lat": waypoint["latitude"], "lon": waypoint["longitude"]}
            waypoints_gps.append(point)
            if "elevation" in waypoint:
                point["ele"] = waypoint["elevation"]
            else:
                point["ele"] = None
            waypoints.append(self._convert_waypoint(point))
        self.waypoints = waypoints
        self.waypoints_gps = waypoints_gps

        if not self.waypoints:
            self.get_logger().error("No waypoints found in YAML file.")
        else:
            self.get_logger().info(f"Parsed {len(waypoints)} waypoints from YAML file.")

    def send_path(self):
        if not self.waypoints:
            return

        if self.args.navigate_through_poses:
            waypoint_msg = NavigateThroughPoses.Goal()
            waypoint_msg.poses.goals = self.waypoints
            waypoint_msg.behavior_tree = (
                "MainTree"  # TODO: could be wrong, taken from VP config
            )
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

    def send_data_url(self, msg_type):
        data = self.data
        if msg_type == "path":
            data["mission"] = {
                "waypoints": self.waypoints_gps,
                "current_waypoint_index": self.current_waypoint,
            }
            if self.pose_gps and self.pose_ekf:
                data["position"] = {"gps": self.pose_gps, "ekf": self.pose_ekf}
            else:
                data["position"] = {"gps": [], "ekf": []}
        elif msg_type == "update":
            data["mission"] = {
                "current_waypoint_index": self.current_waypoint,
            }
            if self.pose_gps and self.pose_ekf:
                data["position"] = {"gps": self.pose_gps, "ekf": self.pose_ekf}
            else:
                data["position"] = {"gps": [], "ekf": []}
        else:
            self.get_logger().error(f"Unknown message type: {msg_type}")

        try:
            response = requests.post(
                SERVER_URL,
                headers={"Content-Type": "application/json"},
                data=json.dumps(data),
            )

            # Check response
            if response.status_code == 200:
                self.get_logger().info("Data sent successfully!")
            elif response.status_code == 202:
                self.get_logger().warn("Failed with status: Missing path.")
                self.send_data_url("path")
            else:
                self.get_logger().error(
                    f"Failed to send data. Status code: {response.status_code}"
                )
                self.get_logger().error(response.text)

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Error sending request: {e}")

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error("Goal rejected by server")
            return

        self.get_logger().info("Goal accepted by server")
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        self.send_data_url("update")
        feedback = feedback_msg.feedback
        if self.args.navigate_through_poses:
            self.get_logger().info(
                f"   CURRENT: pose {feedback.current_pose.pose.position} navigation time {feedback.navigation_time}"
            )
            self.get_logger().info(
                f" REMAINING: number of poses {feedback.number_of_poses_remaining}, distance: {round(feedback.distance_remaining, 2)} m"
            )
            self.get_logger().info(f"RECOVERIES: {feedback.number_of_recoveries}")
            self.current_waypoint = (
                self.number_waypoints - feedback.number_of_poses_remaining
            )
        else:
            self.get_logger().info(
                f"Current waypoint index: {feedback.current_waypoint}"
            )
            self.current_waypoint = feedback.current_waypoint

    def result_callback(self, future):
        result = future.result().result
        if self.args.navigate_through_poses:
            if result.error_msg:
                self.get_logger().warn(f"Error message: {result.error_msg}")
            else:
                self.get_logger().warn("Finished without error")
        else:
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

    def gps_callback(self, msg):
        self.pose_gps = {"lat": msg.latitude, "lon": msg.longitude}
        if not self.path_send_url and self.pose_ekf:
            self.send_data_url("path")
            self.path_send_url = True

    def ekf_callback(self, msg):
        self.pose_ekf = {"lat": msg.latitude, "lon": msg.longitude}
        if not self.path_send_url and self.pose_gps:
            self.send_data_url("path")
            self.path_send_url = True

    def save_waypoint_index(self):
        index_file_path = os.path.join(
            os.path.dirname(self.gps_file),
            "waypoint_index",
            f"{self.get_clock().now()}.txt",
        )

        if not os.path.exists(os.path.dirname(index_file_path)):
            os.makedirs(os.path.dirname(index_file_path))
        with open(index_file_path, "w") as f:
            f.write(str(self.current_waypoint))

        self.get_logger().info(
            f"Saved current waypoint index {self.current_waypoint} to {index_file_path}"
        )


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
        "-r", "--robot", type=str, default="helhest-robot", help="Robot name"
    )
    parser.add_argument(
        "-s", "--start", type=int, default=0, help="Start index of waypoints"
    )
    parser.add_argument(
        "--reverse", action="store_true", help="Reverse the order of waypoints"
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
    time.sleep(1)
    try:
        node.send_path()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        node.cancel_goal()
        node.save_waypoint_index()
        node.destroy_node()


if __name__ == "__main__":
    args = parse_args()
    main(args)
