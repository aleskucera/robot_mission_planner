#!/usr/bin/env python3

import os
import argparse

import utm
import yaml
import json
import gpxpy
import requests

import rospy
import tf2_ros
import tf2_geometry_msgs

from nav_msgs.msg import Path
from std_msgs.msg import Int32
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped

# Define the server URL
SERVER_URL = "http://45.91.169.180:5000/api/update_data"


class GPSFollower:
    def __init__(self, gps_file, args):
        self.args = args
        self.gps_file = gps_file
        self.data = {"robot": args.robot}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.publisher = rospy.Publisher("waypoints_path_map", Path, queue_size=10)
        self.sub_index = rospy.Subscriber(Int32, "/gpx/fix", self.index_callback, 10)
        self.sub_gps = rospy.Subscriber(NavSatFix, "/gpx/fix", self.gps_callback, 10)
        self.sub_ekf = rospy.Subscriber(NavSatFix, "/gpx/fix", self.ekf_callback, 10)

        if not os.path.exists(self.gps_file):
            rospy.logerror(f"GPS file {self.gps_file} does not exist")
            exit(1)

        if self.gps_file.endswith(".gpx"):
            self.parse_gpx_file()
        elif self.gps_file.endswith(".yaml", ".yml"):
            self.parse_yaml_file()
        else:
            rospy.logerror(
                "Unsupported file format. Please provide a .gpx or .yaml file."
            )
            exit(1)

        self.waypoints = self.waypoints[self.args.start :]
        self.number_waypoints = len(self.waypoints)
        rospy.loginfo(
            f"Starting from waypoint index {self.args.start}, total waypoints: {self.number_waypoints}"
        )

        rospy.loginfo("GPSFollower node initialized.")

    def _convert_waypoint(self, waypoint):
        pose = PoseStamped()
        pose.header.frame_id = "utm"

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
                    "ele": waypoint.elevation or None,
                }
                waypoints.append(self._convert_waypoint(point))
                waypoints_gps.append(point)
        except Exception as e:
            rospy.logerror(f"Error parsing GPX file: {e}")
            return []
        self.waypoints = waypoints
        self.waypoints_gps = waypoints_gps

        if not self.waypoints:
            rospy.logerror("No waypoints found in GPX file.")
        else:
            rospy.loginfo(f"Parsed {len(waypoints)} waypoints from GPX file.")

    def parse_yaml_file(self):
        waypoints = []
        waypoints_gps = []
        with open(self.gps_file, "r") as f:
            file_waypoints = yaml.safe_load(f)["waypoints"]
        for waypoint in file_waypoints:
            point = {"lat": waypoint["latitude"], "lon": waypoint["longitude"]}
            if "elevation" in waypoint:
                point["ele"] = waypoint["elevation"]
            else:
                point["ele"] = None
            waypoints.append(self._convert_waypoint(point))
            waypoints_gps.append(point)
        self.waypoints = waypoints
        self.waypoints_gps = waypoints_gps

        if not self.waypoints:
            rospy.logerror("No waypoints found in YAML file.")
        else:
            rospy.loginfo(f"Parsed {len(waypoints)} waypoints from YAML file.")

    def send_path(self):
        if not self.waypoints:
            return

        waypoints = []
        transform = self.tf_buffer.lookup_transform(
            "map", "utm", rospy.Time(0), rospy.Duration(1.0)
        )

        for waypoint in self.waypoints:
            waypoint_transformed = tf2_geometry_msgs.do_transform_pose(
                waypoint, transform
            )

            waypoints.append(waypoint_transformed)

        rospy.loginfo(f"Sending {len(waypoints)} waypoints to follow")
        pose_array = Path()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = rospy.Time.now()
        pose_array.poses = waypoints

        self.publisher.publish(pose_array)

        self.send_data_url("path")

    def send_data_url(self, msg_type):
        data = self.data
        if msg_type == "path":
            data["mission"] = {
                "waypoints": self.waypoints_gps,
                "current_waypoint_index": 0,
            }
            data["position"] = {"gps": self.pose_gps, "ekf": self.pose_ekf}
        elif msg_type == "update":
            data["mission"] = {
                "waypoints": [],
                "current_waypoint_index": self.current_waypoint,
            }
            data["position"] = {"gps": self.pose_gps, "ekf": self.pose_ekf}
        else:
            rospy.logerr(f"Unknown message type: {msg_type}")

        try:
            response = requests.post(
                SERVER_URL,
                headers={"Content-Type": "application/json"},
                data=json.dumps(data),
            )

            # Check response
            if response.status_code == 200:
                rospy.loginfo("Data sent successfully!")
            elif response.status_code == 202:
                rospy.logwarn("Failed with status: Missing path.")
                self.send_data_url("path")
            else:
                rospy.logerr(
                    f"Failed to send data. Status code: {response.status_code}"
                )

        except requests.exceptions.RequestException as e:
            rospy.logerr(f"Error sending request: {e}")

    def cancel_goal(self):
        pose_array = Path()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = rospy.Time.now()
        pose_array.poses = []

        self.publisher.publish(pose_array)

    def index_callback(self, msg):
        self.current_waypoint = msg.data

    def gps_callback(self, msg):
        self.pose_gps = {"lat": msg.latitude, "lon": msg.longitude}

    def ekf_callback(self, msg):
        self.pose_ekf = {"lat": msg.latitude, "lon": msg.longitude}


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
        "-r", "--robot", type=str, default="marv-robot", help="Robot name"
    )
    parser.add_argument(
        "-s", "--start", type=int, default=0, help="Start index of waypoints"
    )

    return parser.parse_args()


def main(args):
    gpx_file_path = args.file
    gpx_file_path = os.path.join(os.path.dirname(__file__), "../", gpx_file_path)
    if not os.path.exists(gpx_file_path):
        print(f"GPX file {gpx_file_path} does not exist")
        return

    rospy.init_node("gps_follower", anonymous=True)
    gps_follower = GPSFollower(gpx_file_path, args)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down GPS Follower node")
        gps_follower.cancel_goal()


if __name__ == "__main__":
    args = parse_args()
    main(args)
