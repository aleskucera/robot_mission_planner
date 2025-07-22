#!/usr/bin/env python3

import os
import argparse

import utm
import yaml
import json
import gpxpy
import requests
import numpy as np

import rospy
import tf2_ros
import tf2_geometry_msgs

from nav_msgs.msg import Path
from std_msgs.msg import Int32
from nav_msgs.srv import GetPlan
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Vector3, TransformStamped

# Define the server URL
SERVER_URL = "http://45.91.169.180:5001/api/update_data"


class GPSFollower:
    def __init__(self, gps_file, args):
        self.args = args
        self.gps_file = gps_file
        self.data = {"robot_id": args.robot}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.waypoint_dist = rospy.get_param("~waypoint_dist", 2.5)
        self.tolerance = rospy.get_param("~tolerance", 32)
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.get_plan = None
        self.wait_for_get_plan()

        self.publisher = rospy.Publisher("waypoints_route", Path, queue_size=10)
        self.end_point_pub = rospy.Publisher("end_point", PoseStamped, queue_size=1)
        # self.sub_index = rospy.Subscriber("", Int32, self.index_callback, 10)  # TODO:
        self.sub_gps = rospy.Subscriber(
            "gnss/septentrio/raw/fix", NavSatFix, self.gps_callback, queue_size=10
        )
        self.sub_ekf = rospy.Subscriber(
            "gps/filtered", NavSatFix, self.ekf_callback, queue_size=10
        )

        self.pose_gps = None
        self.pose_ekf = None
        self.new_waypoint = True

        if not os.path.exists(self.gps_file):
            rospy.logerror("GPS file %s does not exist", self.gps_file)
            exit(1)

        if self.gps_file.endswith(".gpx"):
            self.parse_gpx_file()
        elif self.gps_file.endswith((".yaml", ".yml")):
            self.parse_yaml_file()
        else:
            rospy.logerror(
                "Unsupported file format. Please provide a .gpx or .yaml file."
            )
            exit(1)

        if args.reverse:
            self.waypoints.reverse()
            self.waypoints_gps.reverse()
        self.waypoints = self.waypoints[self.args.start :]
        self.waypoints_gps = self.waypoints_gps[self.args.start :]
        self.number_waypoints = len(self.waypoints)
        self.current_waypoint = 0

        rospy.loginfo(
            "Starting from waypoint index %s, total waypoints: %s",
            self.args.start,
            self.number_waypoints,
        )

        rospy.loginfo("GPSFollower node initialized.")

    def wait_for_get_plan(self):
        """
        Wait for GetPlan service to be available.
        """
        if self.get_plan is not None:
            return
        rospy.loginfo("Waiting for service get_plan")
        rospy.wait_for_service("get_plan")
        self.get_plan = rospy.ServiceProxy("get_plan", GetPlan)
        rospy.logwarn("Using GetPlan service: %s", self.get_plan.resolved_name)

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
                }
                waypoints_gps.append(point)
                point["ele"] = waypoint.elevation or None
                waypoints.append(self._convert_waypoint(point))
        except Exception as e:
            rospy.logerror("Error parsing GPX file: %s", e)
            return []
        self.waypoints = waypoints
        self.waypoints_gps = waypoints_gps

        if not self.waypoints:
            rospy.logerror("No waypoints found in GPX file.")
        else:
            rospy.loginfo("Parsed %s waypoints from GPX file.", len(waypoints))

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
            rospy.logerror("No waypoints found in YAML file.")
        else:
            rospy.loginfo("Parsed %s waypoints from YAML file.", len(waypoints))

    def send_path(self):
        if not self.waypoints:
            return
        if self.get_plan is None:
            rospy.logwarn("Send path, but get_plan is None.")
            rospy.loginfo("Waiting for service get_plan")
            rospy.wait_for_service("get_plan")
            self.get_plan = rospy.ServiceProxy("get_plan", GetPlan)
            rospy.logwarn("Using GetPlan service: %s", self.get_plan.resolved_name)
        self.send_data_url("path")

        waypoints = []
        transform = self.tf_buffer.lookup_transform(
            "map", "utm", rospy.Time(0), rospy.Duration(1.0)
        )

        if not self.args.naex:
            for waypoint in self.waypoints:
                waypoint_transformed = tf2_geometry_msgs.do_transform_pose(
                    waypoint, transform
                )

                waypoints.append(waypoint_transformed)

            rospy.loginfo("Sending %s waypoints to follow", len(waypoints))
            pose_array = Path()
            pose_array.header.frame_id = self.target_frame
            pose_array.header.stamp = rospy.Time.now()
            pose_array.poses = waypoints

            self.publisher.publish(pose_array)
        else:
            if not self.new_waypoint:
                return
            self.new_waypoint = False
            if self.args.waypoint_sleep != 0.0:
                rospy.loginfo(
                    "Waypoint reached, waiting for %s seconds to localize robot"
                )
                rospy.sleep(self.args.waypoint_sleep)

            rospy.loginfo(
                "point "
                + str(self.current_waypoint)
                + " waypoints: "
                + str(len(waypoints))
            )
            waypoint_transformed = tf2_geometry_msgs.do_transform_pose(
                self.waypoints[self.current_waypoint], transform
            )

            target_point = (
                waypoint_transformed.pose.position.x,
                waypoint_transformed.pose.position.y,
                0,
            )

            # Create goal pose
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = self.target_frame
            goal.pose.position = Point(*target_point)
            goal.pose.orientation.w = 1

            # Create start pose
            start = PoseStamped()
            start.header.stamp = rospy.Time.now()
            start.header.frame_id = self.target_frame
            start.pose.position = Point(
                float("nan"), float("nan"), float("nan")
            )  # Use current position
            start.pose.orientation.w = 1

            self.end_point_pub.publish(goal)

            try:
                rospy.loginfo("Sending path to service")
                res = self.get_plan(start, goal, self.tolerance)
            except rospy.ServiceException as e:
                rospy.logerr("GetPlan service failed: %s", e)
                self.get_plan = None
                return

            if len(res.plan.poses) == 0:
                rospy.logwarn("No plan found.")
                return

    def send_data_url(self, msg_type):
        data = self.data
        rospy.loginfo("Sending data")
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
            rospy.logerr("Unknown message type: %s", msg_type)

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
                    "Failed to send data. Status code: %s", response.status_code
                )
                rospy.logerr(response.text)

        except requests.exceptions.RequestException as e:
            rospy.logerr("Error sending request: %s", e)

    def cancel_goal(self):
        pose_array = Path()
        pose_array.header.frame_id = self.target_frame
        pose_array.header.stamp = rospy.Time.now()
        pose_array.poses = []

        self.publisher.publish(pose_array)

    def index_callback(self, msg):
        self.current_waypoint = msg.data

    def gps_callback(self, msg):
        self.pose_gps = {"lat": msg.latitude, "lon": msg.longitude}

    def ekf_callback(self, msg):
        self.pose_ekf = {"lat": msg.latitude, "lon": msg.longitude}

        curr_pos = utm.from_latlon(self.pose_ekf["lat"], self.pose_ekf["lon"])[:2]
        curr_way = np.array(
            (
                self.waypoints[self.current_waypoint].pose.position.x,
                self.waypoints[self.current_waypoint].pose.position.y,
            )
        )
        dist = np.linalg.norm(curr_pos - curr_way)
        rospy.loginfo("Distance to current waypoint: %s", dist)
        if dist < self.waypoint_dist:
            self.current_waypoint += 1
            self.new_waypoint = True
        if self.current_waypoint >= len(self.waypoints):
            rospy.loginfo("Reached the last waypoint.")

    def send_test_path(self):
        target_point = [0.0, 7.0, 0.0]

        # Create goal pose
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.target_frame
        goal.pose.position = Point(*target_point)
        goal.pose.orientation.w = 1

        # Create start pose
        start = PoseStamped()
        start.header.stamp = rospy.Time.now()
        start.header.frame_id = self.target_frame
        start.pose.position = Point(
            float("nan"), float("nan"), float("nan")
        )  # Use current position
        start.pose.orientation.w = 1
        try:
            res = self.get_plan(start, goal, self.tolerance)
        except rospy.ServiceException as e:
            rospy.logerr("GetPlan service failed: %s", e)
            self.get_plan = None
            return

        if len(res.plan.poses) == 0:
            rospy.logwarn("No plan found")
            return


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
    parser.add_argument(
        "--naex", action="store_true", help="Use NAEX as path planner and follower"
    )
    parser.add_argument(
        "--reverse", action="store_true", help="Reverse the order of waypoints"
    )
    parser.add_argument(
        "--waypoint_sleep",
        type=float,
        default=0,
        help="Wait time on each waypoint, before moving to the next",
    )

    return parser.parse_args()


def main(args):
    gpx_file_path = args.file
    gpx_file_path = os.path.join(os.path.dirname(__file__), "../", gpx_file_path)
    if not os.path.exists(gpx_file_path):
        print("GPX file %s does not exist", gpx_file_path)
        return

    rospy.init_node("gps_follower", anonymous=True)
    gps_follower = GPSFollower(gpx_file_path, args)
    while not rospy.is_shutdown():
        gps_follower.send_path()
        rospy.sleep(1.0)


if __name__ == "__main__":
    args = parse_args()
    main(args)
