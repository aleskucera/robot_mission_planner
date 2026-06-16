#!/usr/bin/env python3

import os
import time
import math
import utm
import yaml
import json
import gpxpy
import requests
import numpy as np
from ros2_numpy import numpify
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseArray, PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPose
from nav2_msgs.action import NavigateToPose, FollowWaypoints, FollowGPSWaypoints

from tf2_ros import Buffer, TransformListener
from action_msgs.msg import GoalStatus

# Define the server URL for telemetry
SERVER_URL = "http://45.91.169.180:5001/api/update_data"


class RoadFollower(Node):
    def __init__(self):
        super().__init__("road_follower")

        # --- States ---
        self.STATE_ROAD = 0
        self.STATE_GPS = 1
        self.state = self.STATE_ROAD
        self._active_intersection = None

        # --- Parameters (Road Following) ---
        self.declare_parameter("road_points_topic", "/road_points")
        road_points_topic = (
            self.get_parameter("road_points_topic").get_parameter_value().string_value
        )

        # --- Parameters (GPS Following) ---
        self.declare_parameter("file", "")
        self.declare_parameter("robot_id", "helhest-robot")
        self.declare_parameter("start", 0)
        self.declare_parameter("reverse", False)
        self.declare_parameter("loop", True)
        self.declare_parameter("use_utm", True)
        self.declare_parameter("utm_frame", "utm")
        self.declare_parameter("local_frame", "local_utm")

        # --- Parameters (Thresholds) ---
        self.declare_parameter(
            "intersection_enter_threshold", 3.0
        )  # metry před křižovatkou přepnutí followerů
        self.declare_parameter(
            "intersection_exit_threshold", 4.0
        )  # vzdalenost od krizovatky pro ukoncení gps followeru
        self.declare_parameter(
            "gps_goal_threshold", 3.0
        )  # vzdalenost od cile pro ukončení gps followeru
        self.declare_parameter(
            "lookahead_sync_window", 15
        )  # velikost okna pro synchronizaci indexu waypointu k nejblizsimu budoucimu bodu

        self.gps_file_name = self.get_parameter("file").get_parameter_value().string_value
        self.robot_id = self.get_parameter("robot_id").get_parameter_value().string_value
        self.start_index = self.get_parameter("start").get_parameter_value().integer_value
        self.reverse = self.get_parameter("reverse").get_parameter_value().bool_value
        self.loop = self.get_parameter("loop").get_parameter_value().bool_value
        self.use_utm = self.get_parameter("use_utm").get_parameter_value().bool_value
        self.utm_frame = self.get_parameter("utm_frame").get_parameter_value().string_value
        self.local_frame = self.get_parameter("local_frame").get_parameter_value().string_value

        self.enter_threshold = (
            self.get_parameter("intersection_enter_threshold").get_parameter_value().double_value
        )
        self.exit_threshold = (
            self.get_parameter("intersection_exit_threshold").get_parameter_value().double_value
        )
        self.gps_threshold = (
            self.get_parameter("gps_goal_threshold").get_parameter_value().double_value
        )
        self.lookahead_sync_window = (
            self.get_parameter("lookahead_sync_window").get_parameter_value().integer_value
        )

        # --- TF Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.utm_to_local = None
        self._tf_ready = False

        # --- Action Clients ---
        self._road_action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        if not self.use_utm:
            self._gps_action_client = ActionClient(
                self, FollowGPSWaypoints, "follow_gps_waypoints"
            )  # netestováno
        else:
            self._gps_action_client = ActionClient(self, FollowWaypoints, "follow_waypoints")

        # --- Publishers ---
        self._marker_pub = self.create_publisher(MarkerArray, "gps_waypoints_markers", 10)
        self._marker_timer = self.create_timer(5.0, self._publish_waypoints_markers)

        # Initialize indices before loading data to avoid AttributeError in timers/markers
        self.current_waypoint_index = self.start_index
        self.number_waypoints = 0

        # Try to fetch transformation asynchronously if using UTM
        if self.use_utm:
            self.get_logger().info("try to get utm_to_local transform")
            self._utm_timer = self.create_timer(1.0, self.get_utm_to_local)

        else:
            self.get_logger().info("NOT try to get utm_to_local transform")
            self._utm_timer = None
            self._tf_ready = True

        # --- GPS Data Loading ---
        self.waypoints = []  # List of PoseStamped (UTM) or GeoPose
        self.waypoints_raw = []  # List of dicts with lat, lon, ele
        self.gps_path = ""
        self._load_gps_data()

        self.number_waypoints = len(self.waypoints)

        # --- Variables ---
        self._latest_road_path = None
        self._intersections = None
        self._goal_handle = None
        self._goal_active = False
        self._threshold_triggered = False
        self._pending_goal_timer = None
        self._gps_start_index = 0

        self.pose_gps = None
        self.pose_ekf = None
        self.path_send_url = False
        self.data = {"robot_id": self.robot_id}

        # --- Subscriptions ---
        self._sub_road = self.create_subscription(Path, road_points_topic, self._path_callback, 10)
        self._sub_intersections = self.create_subscription(
            PoseArray, "/intersections", self._intersections_callback, 10
        )
        self._sub_gps = self.create_subscription(NavSatFix, "/gps/fix", self._gps_callback, 10)
        self._sub_ekf = self.create_subscription(NavSatFix, "/gps/filtered", self._ekf_callback, 10)

        self.get_logger().info("Waiting for action servers...")
        self._road_action_client.wait_for_server()
        self._gps_action_client.wait_for_server()

        self.get_logger().info(
            f"Road Follower with GPS logic initialized.\n"
            f"Road topic: {road_points_topic}, GPS file: {self.gps_file_name}\n"
            f"Thresholds: Enter={self.enter_threshold}m, Exit={self.exit_threshold}m, GPS={self.gps_threshold}m"
        )

        self.create_timer(1.0, self._main_logic_step)

    def get_utm_to_local(self) -> None:
        """
        Pokusí se získat transformaci z UTM do local framu.
        Po úspěchu aplikuje transformaci na načtené waypointy a zruší timer.
        """
        try:
            utm_to_local = self.tf_buffer.lookup_transform(
                self.local_frame,
                self.utm_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1.0),
            )
            self.utm_to_local = numpify(utm_to_local.transform)
            self.get_logger().info(f"Got UTM to local transform:\n{self.utm_to_local}")

            self._tf_ready = True
            self._process_waypoints()  # Zpracuj body, když známe transformaci

            if self._utm_timer:
                self._utm_timer.cancel()
                self._utm_timer = None
        except Exception as e:
            self.get_logger().warn(f"Failed to get UTM to local transform: {e}")

    def _transform_point(self, x: float, y: float, z: float = 0.0):
        """Aplikuje transformační matici na bod (stejný princip jako osm_intersections)."""
        if self.utm_to_local is None:
            return x, y, z

        point = np.array([x, y, z]).reshape(3, 1)
        res = np.dot(self.utm_to_local[:3, :3], point) + self.utm_to_local[:3, 3:]
        return res[0][0], res[1][0], res[2][0]

    def _process_waypoints(self):
        """Převede hrubé GPS souřadnice do lokálního framu po načtení TF."""
        self.waypoints = []
        for pt in self.waypoints_raw:
            self.waypoints.append(self._convert_to_msg(pt))
        self.number_waypoints = len(self.waypoints)
        self.get_logger().info(
            f"Successfully processed {self.number_waypoints} waypoints into {self.local_frame} frame."
        )

    def _load_gps_data(self):
        """Loads and parses the GPS file (GPX or YAML)."""
        if self.gps_file_name == "":
            self.get_logger().warn("No GPS file specified. GPS following will not be available.")
            return

        # Support absolute paths or relative to robot_mission_planner/data
        if os.path.isabs(self.gps_file_name):
            self.gps_path = self.gps_file_name
        else:
            data_dir = os.path.join(os.path.dirname(__file__), "..", "data")
            self.gps_path = os.path.join(data_dir, self.gps_file_name)

        if not os.path.exists(self.gps_path):
            self.get_logger().error(f"GPS file {self.gps_path} does not exist!")
            return

        try:
            if self.gps_path.endswith(".gpx"):
                with open(self.gps_path, "r") as f:
                    gpx = gpxpy.parse(f)
                for wp in gpx.waypoints:
                    point = {"lat": wp.latitude, "lon": wp.longitude, "ele": wp.elevation or 0.0}
                    self.waypoints_raw.append(point)
                    if self._tf_ready:
                        self.waypoints.append(self._convert_to_msg(point))
            elif self.gps_path.endswith((".yaml", ".yml")):
                with open(self.gps_path, "r") as f:
                    data = yaml.safe_load(f)
                for wp in data.get("waypoints", []):
                    point = {
                        "lat": wp["latitude"],
                        "lon": wp["longitude"],
                        "ele": wp.get("elevation", 0.0),
                    }
                    self.waypoints_raw.append(point)
                    if self._tf_ready:
                        self.waypoints.append(self._convert_to_msg(point))

            if self.reverse:
                # self.waypoints.reverse()
                self.waypoints_raw.reverse()

            self.get_logger().info(
                f"Loaded {len(self.waypoints_raw)} waypoints from {self.gps_file_name}"
            )

            if self._tf_ready:
                self._process_waypoints()

        except Exception as e:
            self.get_logger().error(f"Failed to parse GPS file: {e}")

    def _publish_waypoints_markers(self):
        """Publishes loaded waypoints as Markers for visualization in RViz."""
        if not self.waypoints_raw or not self._tf_ready:
            return

        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, pt in enumerate(self.waypoints_raw):
            marker = Marker()
            marker.header.frame_id = self.local_frame if self.use_utm else self.utm_frame
            marker.header.stamp = now
            marker.ns = "gps_waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Convert to UTM and apply transformation matrix from TF
            try:
                utm_coords = utm.from_latlon(pt["lat"], pt["lon"])
                tx, ty, tz = self._transform_point(utm_coords[0], utm_coords[1], pt.get("ele", 0.0))
                marker.pose.position.x = float(tx)
                marker.pose.position.y = float(ty)
                marker.pose.position.z = float(tz)
                # self.get_logger().info(f"{utm_coords[0]}, {tx}")

            except Exception as e:
                self.get_logger().warn(
                    f"Failed to convert point {i} to local frame for marker: {e}"
                )
                continue

            # Highlight current waypoint in green, others in yellow
            marker.scale.x = 4.0
            marker.scale.y = 4.0
            marker.scale.z = 4.0
            marker.color.a = 0.8
            if i == self.current_waypoint_index:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0

            marker_array.markers.append(marker)

        # self.get_logger().info("Waypoints published.")

        self._marker_pub.publish(marker_array)

    def _convert_to_msg(self, point):
        """Converts raw GPS point to ROS message based on use_utm."""
        if not self.use_utm:
            msg = GeoPose()
            msg.position.latitude = point["lat"]
            msg.position.longitude = point["lon"]
            msg.position.altitude = point["ele"]
            return msg
        else:
            msg = PoseStamped()
            msg.header.frame_id = self.local_frame
            utm_coords = utm.from_latlon(point["lat"], point["lon"])
            tx, ty, tz = self._transform_point(utm_coords[0], utm_coords[1], point["ele"])
            msg.pose.position.x = float(tx)
            msg.pose.position.y = float(ty)
            msg.pose.position.z = float(tz)
            return msg

    def _get_robot_pose_in_map(self):
        """Returns the current robot pose in the 'map' frame."""
        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            return trans.transform.translation
        except Exception:
            return None

    def _intersections_callback(self, msg):
        self._intersections = msg

    def _path_callback(self, msg):
        """Handles incoming road points."""
        if not msg.poses:
            return
        self._latest_road_path = msg

        # Pokud jsme v režimu silnice a přijde nová cesta, rovnou pošleme cíl
        if self.state == self.STATE_ROAD:
            self._send_road_goal()

    def _gps_callback(self, msg):
        self.pose_gps = {"lat": msg.latitude, "lon": msg.longitude}
        if not self.path_send_url and self.pose_ekf:
            self.send_data_url("path")
            self.path_send_url = True

    def _ekf_callback(self, msg):
        self.pose_ekf = {"lat": msg.latitude, "lon": msg.longitude}
        if not self.path_send_url and self.pose_gps:
            self.send_data_url("path")
            self.path_send_url = True

    def send_data_url(self, msg_type):
        """Sends telemetry data to the remote server."""
        data = self.data
        if msg_type == "path":
            data["mission"] = {
                "waypoints": self.waypoints_raw,
                "current_waypoint_index": self.current_waypoint_index,
            }
            if self.pose_gps and self.pose_ekf:
                data["position"] = {"gps": self.pose_gps, "ekf": self.pose_ekf}
            else:
                data["position"] = {"gps": [], "ekf": []}
        elif msg_type == "update":
            data["mission"] = {
                "current_waypoint_index": self.current_waypoint_index,
            }
            if self.pose_gps and self.pose_ekf:
                data["position"] = {"gps": self.pose_gps, "ekf": self.pose_ekf}
            else:
                data["position"] = {"gps": [], "ekf": []}

        try:
            response = requests.post(
                SERVER_URL,
                headers={"Content-Type": "application/json"},
                data=json.dumps(data),
                timeout=1.0,
            )
            if response.status_code == 202:
                self.send_data_url("path")
        except Exception:
            pass

    def _main_logic_step(self):
        """Core logic to update virtual tracking and handle state transitions."""
        robot_pos = self._get_robot_pose_in_map()
        if robot_pos is None:
            return

        # 1. Update Virtual GPS Tracking
        self._update_virtual_gps(robot_pos)

        # 2. Check for State Transitions
        if self._intersections is not None:
            self._check_state_transitions(robot_pos)

    def _update_virtual_gps(self, robot_pos):
        """Updates current_waypoint_index by checking distance to next waypoint."""
        if self.state == self.STATE_GPS:
            return

        self._sync_waypoint_index_to_closest()

    def _sync_waypoint_index_to_closest(self):
        """
        Synchronizes self.current_waypoint_index to the closest waypoint in a lookahead window
        to prevent getting stuck on missed waypoints.
        """
        if not self.waypoints:
            return

        num_wps = len(self.waypoints)

        # Get robot position
        rob_x, rob_y = None, None
        if self.use_utm:
            try:
                trans_local = self.tf_buffer.lookup_transform(
                    self.local_frame, "base_link", rclpy.time.Time()
                )
                rob_x = trans_local.transform.translation.x
                rob_y = trans_local.transform.translation.y
            except Exception as e:
                self.get_logger().warn(f"Failed to get robot pose for waypoint sync: {e}")
                return
        else:
            if self.pose_gps:
                rob_x = self.pose_gps["lat"]
                rob_y = self.pose_gps["lon"]
            else:
                return

        # Collect distances for the next lookahead_sync_window waypoints
        best_idx = self.current_waypoint_index
        min_dist = float("inf")

        for offset in range(self.lookahead_sync_window):
            idx = self.current_waypoint_index + offset
            if idx >= num_wps:
                if self.loop:
                    idx = idx % num_wps
                else:
                    break

            # Calculate distance
            if self.use_utm:
                target = self.waypoints[idx].pose.position
                dist = math.sqrt((rob_x - target.x) ** 2 + (rob_y - target.y) ** 2)
            else:
                target = self.waypoints_raw[idx]
                d_lat = (rob_x - target["lat"]) * 111320
                d_lon = (
                    (rob_y - target["lon"])
                    * 111320
                    * math.cos(math.radians(target["lat"]))
                )
                dist = math.sqrt(d_lat**2 + d_lon**2)

            if dist < min_dist:
                min_dist = dist
                best_idx = idx

        # If we found a closer waypoint further along, update the index
        if best_idx != self.current_waypoint_index:
            self.get_logger().info(
                f"Waypoint Sync: Skipping missed waypoints. "
                f"Moving index from {self.current_waypoint_index} to closest: {best_idx} (dist: {min_dist:.2f}m)"
            )
            self.current_waypoint_index = best_idx

        # Normal progression: if we are within self.gps_threshold of the closest waypoint, advance to the next one
        if min_dist < self.gps_threshold:
            next_idx = self.current_waypoint_index + 1
            if next_idx >= num_wps:
                if self.loop:
                    next_idx = 0
                else:
                    next_idx = num_wps - 1  # Stay on the last waypoint if not looping

            if next_idx != self.current_waypoint_index:
                self.get_logger().info(
                    f"Virtual GPS: Reached waypoint {self.current_waypoint_index} (dist: {min_dist:.2f}m < {self.gps_threshold}m). "
                    f"Advancing index to {next_idx}."
                )
                self.current_waypoint_index = next_idx

    def _check_state_transitions(self, robot_pos):
        """Checks if we should switch between ROAD and GPS states."""
        if self.state == self.STATE_ROAD:
            closest_dist = float("inf")
            closest_idx = -1
            for i, pose in enumerate(self._intersections.poses):
                d = math.sqrt(
                    (robot_pos.x - pose.position.x) ** 2 + (robot_pos.y - pose.position.y) ** 2
                )
                if d < closest_dist:
                    closest_dist = d
                    closest_idx = i

            if closest_dist < self.enter_threshold:
                self.get_logger().info(
                    f"Approaching intersection ({closest_dist:.2f}m). Switching to GPS mode."
                )
                self.state = self.STATE_GPS
                self._active_intersection = self._intersections.poses[closest_idx]
                self._cancel_current_goal()
                self._schedule_goal(delay_sec=1.0, mode="GPS")

        elif self.state == self.STATE_GPS:
            # Find distance to the closest intersection to ensure we clear all nearby crossroads
            closest_inter_dist = float("inf")
            if self._intersections is not None and len(self._intersections.poses) > 0:
                for pose in self._intersections.poses:
                    d = math.sqrt(
                        (robot_pos.x - pose.position.x) ** 2 + (robot_pos.y - pose.position.y) ** 2
                    )
                    if d < closest_inter_dist:
                        closest_inter_dist = d
            else:
                # Fallback to the active intersection if the topic list is empty/None
                if self._active_intersection is not None:
                    closest_inter_dist = math.sqrt(
                        (robot_pos.x - self._active_intersection.position.x) ** 2
                        + (robot_pos.y - self._active_intersection.position.y) ** 2
                    )

            if closest_inter_dist > self.exit_threshold:
                target_reached = False
                if self.use_utm:
                    try:
                        trans_local = self.tf_buffer.lookup_transform(
                            self.local_frame, "base_link", rclpy.time.Time()
                        )
                        rob_local = trans_local.transform.translation
                        target = self.waypoints[self.current_waypoint_index].pose.position
                        dist_target = math.sqrt(
                            (rob_local.x - target.x) ** 2 + (rob_local.y - target.y) ** 2
                        )
                        if dist_target < self.gps_threshold:
                            target_reached = True
                    except Exception:
                        pass
                else:
                    if self.pose_gps:
                        target = self.waypoints_raw[self.current_waypoint_index]
                        d_lat = (self.pose_gps["lat"] - target["lat"]) * 111320
                        d_lon = (
                            (self.pose_gps["lon"] - target["lon"])
                            * 111320
                            * math.cos(math.radians(target["lat"]))
                        )
                        dist_target = math.sqrt(d_lat**2 + d_lon**2)
                        if dist_target < self.gps_threshold:
                            target_reached = True

                if target_reached:
                    self.get_logger().info(
                        f"Passed all nearby intersections (closest: {closest_inter_dist:.2f}m). Switching back to ROAD mode."
                    )
                    self.state = self.STATE_ROAD
                    self._active_intersection = None
                    self._cancel_current_goal()
                    self._schedule_goal(delay_sec=1.0, mode="ROAD")

    def _cancel_current_goal(self):
        """Cancels any active goal (Road or GPS)."""
        if self._goal_handle is not None:
            self.get_logger().info("Cancelling current goal for state transition.")
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._goal_active = False

    def _schedule_goal(self, delay_sec, mode):
        """Schedules a goal update after a delay."""
        if self._pending_goal_timer:
            self._pending_goal_timer.cancel()

        if mode == "GPS":
            self._pending_goal_timer = self.create_timer(delay_sec, self._send_gps_goal_timer_cb)
        else:
            self._pending_goal_timer = self.create_timer(delay_sec, self._send_road_goal_timer_cb)

    def _send_road_goal_timer_cb(self):
        self._pending_goal_timer.cancel()
        self._pending_goal_timer = None
        self._send_road_goal()

    def _send_gps_goal_timer_cb(self):
        self._pending_goal_timer.cancel()
        self._pending_goal_timer = None
        self._send_gps_goal()

    def _send_road_goal(self):
        """Sends a goal to NavigateToPose using the latest road points."""
        if (
            self.state != self.STATE_ROAD
            or self._latest_road_path is None
            or not self._latest_road_path.poses
        ):
            return

        pose_stamped = self._latest_road_path.poses[-1]
        pose_stamped.header.stamp = self.get_clock().now().to_msg()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        self.get_logger().info(
            f"Road Goal: sending to ({pose_stamped.pose.position.x:.2f}, {pose_stamped.pose.position.y:.2f})"
        )
        self._goal_active = True
        self._threshold_triggered = False

        self.send_goal_future = self._road_action_client.send_goal_async(
            goal_msg, feedback_callback=self._road_feedback_callback
        )
        self.send_goal_future.add_done_callback(self._goal_response_callback)

    def _send_gps_goal(self):
        """Sends a goal to FollowWaypoints/FollowGPSWaypoints."""
        if self.state != self.STATE_GPS or not self.waypoints:
            return

        self._sync_waypoint_index_to_closest()

        remaining_waypoints = self.waypoints[self.current_waypoint_index :]
        if not remaining_waypoints:
            self.get_logger().warn("No remaining GPS waypoints to send.")
            return

        self._gps_start_index = self.current_waypoint_index

        if not self.use_utm:
            goal_msg = FollowGPSWaypoints.Goal()
            goal_msg.gps_poses = remaining_waypoints
        else:
            goal_msg = FollowWaypoints.Goal()
            goal_msg.poses = remaining_waypoints

        self.get_logger().info(f"GPS Goal: sending {len(remaining_waypoints)} waypoints.")
        self._goal_active = True

        self.send_goal_future = self._gps_action_client.send_goal_async(
            goal_msg, feedback_callback=self._gps_feedback_callback
        )
        self.send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server.")
            self._goal_active = False
            return

        self.get_logger().info("Goal accepted.")
        self.result_future = self._goal_handle.get_result_async()
        self.result_future.add_done_callback(self._result_callback)

    def _road_feedback_callback(self, feedback_msg):
        """Handles feedback during Road Following."""
        dist = feedback_msg.feedback.distance_remaining
        if dist == 0.0:
            return

        if dist < self.gps_threshold and not self._threshold_triggered:
            self._threshold_triggered = True
            self.get_logger().info(
                f"Road distance threshold reached ({dist:.2f}m). Preparing next goal."
            )
            self._goal_active = False

    def _gps_feedback_callback(self, feedback_msg):
        """Handles feedback during GPS Following."""
        rel_idx = feedback_msg.feedback.current_waypoint
        new_global_idx = self._gps_start_index + rel_idx

        if new_global_idx != self.current_waypoint_index:
            self.get_logger().info(f"GPS Feedback: Reached waypoint {new_global_idx}")
            self.current_waypoint_index = new_global_idx

    def _result_callback(self, future):
        status = future.result().status
        self.get_logger().info(f"Goal finished with status: {status}")
        self._goal_active = False

        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.state == self.STATE_GPS:
                if self.loop:
                    self.current_waypoint_index = 0
                    self._send_gps_goal()
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("Goal aborted.")

    def save_waypoint_index(self):
        """Saves current waypoint index to a file on shutdown."""
        if not self.gps_path:
            return
        try:
            index_file_path = os.path.join(
                os.path.dirname(self.gps_path), "waypoint_index", f"{int(time.time())}.txt"
            )
            if not os.path.exists(os.path.dirname(index_file_path)):
                os.makedirs(os.path.dirname(index_file_path))
            with open(index_file_path, "w") as f:
                f.write(str(self.current_waypoint_index))
            self.get_logger().info(
                f"Saved current waypoint index {self.current_waypoint_index} to {index_file_path}"
            )
        except Exception:
            pass


def main():
    rclpy.init()
    node = RoadFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.save_waypoint_index()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
