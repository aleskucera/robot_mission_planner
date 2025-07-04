#!/usr/bin/env python3

import os
import argparse

import utm
import yaml
import gpxpy
import rospy
import tf2_ros
import tf2_geometry_msgs

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class GPSFollower:
    def __init__(self, gps_file, args):
        self.args = args
        self.gps_file = gps_file

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.publisher = rospy.Publisher("waypoints_path_map", Path, queue_size=10)

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
            rospy.logerror(f"Error parsing GPX file: {e}")
            return []
        self.waypoints = waypoints
        if not self.waypoints:
            rospy.logerror("No waypoints found in GPX file.")
        else:
            rospy.loginfo(f"Parsed {len(waypoints)} waypoints from GPX file.")

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

    def cancel_goal(self):
        pose_array = Path()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = rospy.Time.now()
        pose_array.poses = []

        self.publisher.publish(pose_array)


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
