#!/usr/bin/env python3

import os
import time
import argparse
import copy 

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
    def __init__(self):
        super().__init__("gps_follower")

        self.declare_parameter("file", "")
        self.declare_parameter("robot", "helhest-robot")
        self.declare_parameter("start", 0)
        self.declare_parameter("reverse", False)
        self.declare_parameter("loop", True)
        self.declare_parameter("use_utm", False)
        self.declare_parameter("navigate_through_poses", False)

        self.gps_file = (
            self.get_parameter("file").get_parameter_value().string_value
        )
        self.robot = (
            self.get_parameter("robot").get_parameter_value().string_value
        )
        self.start = (
            self.get_parameter("start").get_parameter_value().integer_value
        )
        self.reverse = (
            self.get_parameter("reverse").get_parameter_value().bool_value
        )
        self.loop = (
            self.get_parameter("loop").get_parameter_value().bool_value
        )
        self.use_utm = (
            self.get_parameter("use_utm").get_parameter_value().bool_value
        )
        self.navigate_through_poses = (
            self.get_parameter("navigate_through_poses").get_parameter_value().bool_value
        )

        self.data = {"robot_id": self.robot}

        if self.navigate_through_poses:
            self._action_client = ActionClient(
                self, NavigateThroughPoses, "navigate_through_poses"
            )
            self.get_logger().error("Running with navigate_through_poses has not been tested out yet... so beware...")
            self.use_utm = True # TODO: WHAT DOES THIS DO? WHY IS IT HERE?
        else:
            if not self.use_utm:
                self._action_client = ActionClient(
                    self, FollowGPSWaypoints, "follow_gps_waypoints"
                )
            else:
                self._action_client = ActionClient(
                    self, FollowWaypoints, "follow_waypoints"
                )
        
        if self.gps_file == "":
            self.get_logger().error("GPS file not specified")
            exit(1)
        else:
            self.gps_file = os.path.join(os.path.dirname(__file__), "../data", self.gps_file)

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

        if self.reverse:
            self.waypoints.reverse()
            self.waypoints_gps.reverse()
        
        self.waypoints = self.waypoints[self.start :]
        self.waypoints_gps = self.waypoints_gps[self.start :]

        self.number_waypoints = len(self.waypoints)
        self.current_waypoint = 0
        self.pose_gps = None
        self.pose_ekf = None
        self.path_send_url = False
        self.get_logger().info(
            f"Starting from waypoint index {self.start}, total waypoints: {self.number_waypoints}"
        )

        # Wait for the action server to be available
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            if self.navigate_through_poses:
                self.get_logger().info(
                    "Waiting for /navigate_through_poses action server..."
                )
            else:
                self.get_logger().info(
                    f"Waiting for {
                        '/follow_gps_waypoints'
                        if not self.use_utm
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
        if not self.use_utm:
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

    def send_path(self, waypoints):
        if not waypoints:
            return

        if self.navigate_through_poses:
            waypoint_msg = NavigateThroughPoses.Goal()
            waypoint_msg.poses.goals = waypoints
            # waypoint_msg.behavior_tree = (
            #     "MainTree"  # TODO: could be wrong, taken from VP config
            # )
        else:
            if not self.use_utm:
                waypoint_msg = FollowGPSWaypoints.Goal()
                waypoint_msg.gps_poses = waypoints
            else:
                waypoint_msg = FollowWaypoints.Goal()
                waypoint_msg.poses = waypoints

        self.get_logger().info(f"Sending {len(waypoints)} waypoints to follow")
        send_goal_future = self._action_client.send_goal_async(
            waypoint_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def send_data_url(self, msg_type):
        return
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
        if self.navigate_through_poses:
            self.get_logger().info(
                f"   CURRENT: pose {feedback.current_pose.pose.position} navigation time {feedback.navigation_time}", throttle_duration_sec=1
            )
            self.get_logger().info(
                f" REMAINING: number of poses {feedback.number_of_poses_remaining}, distance: {round(feedback.distance_remaining, 2)} m", throttle_duration_sec=1
            )
            self.get_logger().info(f"RECOVERIES: {feedback.number_of_recoveries}", throttle_duration_sec=1)
            self.current_waypoint = (
                self.number_waypoints - feedback.number_of_poses_remaining
            )
        else:
            self.get_logger().info(
                f"Current waypoint index: {feedback.current_waypoint}", throttle_duration_sec=1
            )
            self.current_waypoint = feedback.current_waypoint

    def result_callback(self, future):
        result = future.result().result
        if self.navigate_through_poses:
            if result.error_msg:
                self.get_logger().warn(f"Error message: {result.error_msg}")
                if self.loop:
                    self.get_logger().info(f"Starting plan again from the last waypoint.")
                    self.send_path(self.waypoints[self.current_waypoint:])
            else:
                self.get_logger().warn("Finished without error")
        else:
            if result.missed_waypoints:
                self.get_logger().warn(f"Missed waypoints: {result.missed_waypoints}")
                if self.loop:
                    self.get_logger().info(f"Starting plan again from the last waypoint.")
                    self.send_path(self.waypoints[self.current_waypoint:])
            else:
                self.get_logger().info("Successfully navigated all waypoints")
        if self.loop:
            self.current_waypoint = 0
            self.send_path(self.waypoints)
        else:
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

def main():
    rclpy.init()
    node = GPXFollower()
    time.sleep(1)
    try:
        node.send_path(node.waypoints)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        node.cancel_goal()
        node.save_waypoint_index()
        node.destroy_node()


if __name__ == "__main__":
    main()