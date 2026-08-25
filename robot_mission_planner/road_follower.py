#!/usr/bin/env python3
"""
Road follower with GPS fallback at intersections.

State machine
-------------
ROAD  : follow the visually detected road (``road_points_topic``, nav_msgs/Path from
        path_centerline) by sending its last pose as the navigation goal.
GPS   : near an OSM intersection (``intersections_topic``, geometry_msgs/PoseArray from
        map_data/osm_cloud) follow the pre-planned GPX waypoints instead.

Navigation backends (``nav_backend``)
-------------------------------------
commander : the Helhest field stack (crl_commander on the NUC). ROAD goals are published
            as a PoseStamped on ``goal_waypoint_topic`` in *goto* mode; GPS waypoints are
            published as a latched PoseArray on ``goal_sequence_topic`` (in ``earth_frame``,
            ECEF) and the commander is switched to *sequence* mode.
nav2      : Nav2 ``NavigateToPose`` / ``FollowWaypoints`` (``FollowGPSWaypoints`` when
            ``use_utm`` is false).

Frames
------
All geometry is compared in ``map_frame`` (the robot's fixed frame, ``FP_ENU0`` on
Helhest). Intersections and road paths may arrive in any TF-connected frame; they are
transformed with TF. GPX waypoints are converted lat/lon -> ECEF (``earth_frame``) and
transformed into ``map_frame`` through TF (commander backend), or lat/lon -> UTM and then
``utm_frame`` -> ``map_frame`` (nav2 backend with ``use_utm``).
"""

import json
import math
import os
import time

import gpxpy
import numpy as np
import rclpy
import requests
import utm
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import FollowGPSWaypoints, FollowWaypoints, NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from ros2_numpy import numpify
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

# WGS84 (lat/lon -> ECEF for the commander backend; kept local to avoid a map_data dependency)
_WGS84_A = 6378137.0
_WGS84_E2 = (1.0 / 298.257223563) * (2.0 - 1.0 / 298.257223563)


def latlon_to_ecef(lat_deg: float, lon_deg: float, alt_m: float = 0.0) -> tuple[float, float, float]:
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * math.sin(lat) ** 2)
    x = (n + alt_m) * math.cos(lat) * math.cos(lon)
    y = (n + alt_m) * math.cos(lat) * math.sin(lon)
    z = (n * (1.0 - _WGS84_E2) + alt_m) * math.sin(lat)
    return x, y, z


def transform_xyz(matrix: np.ndarray, x: float, y: float, z: float = 0.0) -> tuple[float, float, float]:
    """Apply a 4x4 homogeneous matrix to a point."""
    p = matrix[:3, :3] @ np.array([x, y, z]) + matrix[:3, 3]
    return float(p[0]), float(p[1]), float(p[2])


class RoadFollower(Node):
    STATE_ROAD = 0
    STATE_GPS = 1

    def __init__(self):
        super().__init__("road_follower")

        self.state = self.STATE_ROAD
        self._active_intersection = None

        # --- Backend ---
        self.declare_parameter("nav_backend", "commander")  # "commander" | "nav2"

        # --- Frames ---
        self.declare_parameter("map_frame", "FP_ENU0")  # fixed frame all geometry is compared in
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("earth_frame", "FP_ECEF")  # ECEF frame for GPX waypoints (commander)
        self.declare_parameter("utm_frame", "utm")  # UTM frame for GPX waypoints (nav2 + use_utm)

        # --- Topics / services ---
        self.declare_parameter("road_points_topic", "/predicted_path_ls")
        self.declare_parameter("intersections_topic", "/intersections")
        self.declare_parameter("gps_fix_topic", "/fixposition/odometry_llh")
        self.declare_parameter("gps_filtered_topic", "")  # optional second NavSatFix (telemetry)
        self.declare_parameter("goal_waypoint_topic", "/goal_waypoint")  # commander: operator goal
        self.declare_parameter("goal_sequence_topic", "/goal_sequence")  # commander: latched PoseArray
        self.declare_parameter("commander_state_topic", "/commander/state")
        self.declare_parameter("switch_mode_service", "/crl_commander/switch_mode")
        self.declare_parameter(
            "configure_sequence_service", "/crl_commander/configure_sequence_mode"
        )
        self.declare_parameter("markers_topic", "gps_waypoints_markers")

        # --- GPS following ---
        self.declare_parameter("file", "")
        self.declare_parameter("robot_id", "helhest-robot")
        self.declare_parameter("start", 0)
        self.declare_parameter("reverse", False)
        self.declare_parameter("loop", True)
        self.declare_parameter("use_utm", True)  # nav2 backend only
        self.declare_parameter("telemetry_url", "http://45.91.169.180:5001/api/update_data")

        # --- Thresholds ---
        self.declare_parameter("intersection_enter_threshold", 3.0)  # m: ROAD -> GPS
        self.declare_parameter("intersection_exit_threshold", 4.0)  # m: from all intersections
        self.declare_parameter("gps_goal_threshold", 3.0)  # m: waypoint reached
        self.declare_parameter("lookahead_sync_window", 15)  # waypoints searched for the closest
        self.declare_parameter("road_goal_update_distance", 1.0)  # m: re-send active road goal
        # When true, GPS -> ROAD additionally requires being within gps_goal_threshold of the
        # current waypoint (legacy behaviour; with sparse waypoints this keeps GPS mode long).
        self.declare_parameter("require_waypoint_reached_to_exit_gps", False)

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.nav_backend = gp("nav_backend")
        if self.nav_backend not in ("commander", "nav2"):
            self.get_logger().error(f"Unknown nav_backend '{self.nav_backend}', using 'commander'")
            self.nav_backend = "commander"
        self.map_frame = gp("map_frame")
        self.robot_frame = gp("robot_frame")
        self.earth_frame = gp("earth_frame")
        self.utm_frame = gp("utm_frame")
        self.gps_file_name = gp("file")
        self.robot_id = gp("robot_id")
        self.start_index = gp("start")
        self.reverse = gp("reverse")
        self.loop = gp("loop")
        self.use_utm = gp("use_utm")
        self.telemetry_url = gp("telemetry_url")
        self.enter_threshold = gp("intersection_enter_threshold")
        self.exit_threshold = gp("intersection_exit_threshold")
        self.gps_threshold = gp("gps_goal_threshold")
        self.lookahead_sync_window = gp("lookahead_sync_window")
        self.road_goal_update_distance = gp("road_goal_update_distance")
        self.require_wp_to_exit = gp("require_waypoint_reached_to_exit_gps")

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Matrix that maps waypoint source coordinates (ECEF or UTM) into map_frame.
        self.waypoint_src_frame = (
            self.earth_frame if self.nav_backend == "commander" else self.utm_frame
        )
        self.src_to_map = None
        self._tf_ready = False

        # --- Backend I/O ---
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.commander_mode = None
        if self.nav_backend == "commander":
            from crl_commander.srv import ConfigureSequenceMode, SwitchMode

            self._srv_types = {"switch": SwitchMode, "configure": ConfigureSequenceMode}
            self._pub_goal_waypoint = self.create_publisher(
                PoseStamped, gp("goal_waypoint_topic"), latched
            )
            self._pub_goal_sequence = self.create_publisher(
                PoseArray, gp("goal_sequence_topic"), latched
            )
            self._cli_switch_mode = self.create_client(SwitchMode, gp("switch_mode_service"))
            self._cli_configure_seq = self.create_client(
                ConfigureSequenceMode, gp("configure_sequence_service")
            )
            self.create_subscription(
                String, gp("commander_state_topic"), self._commander_state_callback, 10
            )
        else:
            self._road_action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
            if not self.use_utm:
                self._gps_action_client = ActionClient(
                    self, FollowGPSWaypoints, "follow_gps_waypoints"
                )
            else:
                self._gps_action_client = ActionClient(self, FollowWaypoints, "follow_waypoints")

        self._marker_pub = self.create_publisher(MarkerArray, gp("markers_topic"), 10)
        self.create_timer(5.0, self._publish_waypoints_markers)

        # --- Waypoints ---
        self.current_waypoint_index = self.start_index
        self.number_waypoints = 0
        self.waypoints_raw = []  # dicts lat/lon/ele
        self.waypoints = []  # backend goal messages (PoseStamped in src frame, or GeoPose)
        self.waypoints_map = []  # (x, y) in map_frame for distance checks
        self.gps_path = ""
        self._load_gps_data()

        if self.nav_backend == "nav2" and not self.use_utm:
            self._tf_ready = True  # lat/lon goals, no transform needed
            self._process_waypoints()
        else:
            self._utm_timer = self.create_timer(1.0, self._resolve_waypoint_transform)

        # --- Runtime state ---
        self._latest_road_path = None
        self._intersections = None  # PoseArray as received
        self._intersections_map = None  # np.ndarray (N, 2) in map_frame
        self._goal_handle = None
        self._goal_active = False
        self._threshold_triggered = False
        self._last_road_goal = None  # (x, y) in map_frame of the active ROAD goal
        self._pending_goal_timer = None
        self._gps_start_index = 0
        self._sequence_configured = False
        self._requested_mode = None  # last mode asked of the commander (state topic lags)
        self._waypoints_synced = False  # first sync searches the whole list

        self.pose_gps = None
        self.pose_ekf = None
        self.path_send_url = False
        self.data = {"robot_id": self.robot_id}

        # --- Subscriptions ---
        self.create_subscription(Path, gp("road_points_topic"), self._path_callback, 10)
        self.create_subscription(
            PoseArray, gp("intersections_topic"), self._intersections_callback, latched
        )
        if gp("gps_fix_topic"):
            self.create_subscription(
                NavSatFix, gp("gps_fix_topic"), self._gps_callback, qos_profile_sensor_data
            )
        if gp("gps_filtered_topic"):
            self.create_subscription(
                NavSatFix, gp("gps_filtered_topic"), self._ekf_callback, qos_profile_sensor_data
            )

        if self.nav_backend == "nav2":
            self.get_logger().info("Waiting for Nav2 action servers...")
            self._road_action_client.wait_for_server()
            self._gps_action_client.wait_for_server()
        else:
            if not self._cli_switch_mode.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn(
                    f"Commander service {gp('switch_mode_service')} not available yet; "
                    "mode switches will be retried when needed."
                )

        self.get_logger().info(
            f"Road follower initialised (backend={self.nav_backend}, map_frame={self.map_frame}, "
            f"robot_frame={self.robot_frame}, waypoint frame={self.waypoint_src_frame}).\n"
            f"Road topic: {gp('road_points_topic')}, intersections: {gp('intersections_topic')}, "
            f"GPS file: {self.gps_file_name}\n"
            f"Thresholds: enter={self.enter_threshold} m, exit={self.exit_threshold} m, "
            f"waypoint={self.gps_threshold} m"
        )
        self.create_timer(1.0, self._main_logic_step)

    # ------------------------------------------------------------------ TF helpers
    def _lookup_matrix(self, target: str, source: str, timeout: float = 0.5):
        """4x4 matrix mapping points in ``source`` into ``target``, or None."""
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                target, source, rclpy.time.Time(), rclpy.duration.Duration(seconds=timeout)
            )
        except Exception as e:  # TransformException and friends
            self.get_logger().warn(f"TF {target} <- {source} unavailable: {e}", throttle_duration_sec=5.0)
            return None
        return numpify(tf_msg.transform)

    def _resolve_waypoint_transform(self):
        """Resolve waypoint source frame -> map_frame once, then convert the waypoints."""
        m = self._lookup_matrix(self.map_frame, self.waypoint_src_frame, timeout=1.0)
        if m is None:
            return
        self.src_to_map = m
        self._tf_ready = True
        self._process_waypoints()
        self._utm_timer.cancel()
        self.get_logger().info(
            f"Got {self.waypoint_src_frame} -> {self.map_frame} transform; "
            f"{self.number_waypoints} waypoints placed in {self.map_frame}"
        )

    def _robot_xy(self):
        """Robot position (x, y) in map_frame, or None."""
        m = self._lookup_matrix(self.map_frame, self.robot_frame, timeout=0.2)
        if m is None:
            return None
        return float(m[0, 3]), float(m[1, 3])

    def _pose_to_map(self, pose: PoseStamped):
        """(x, y) of a PoseStamped in map_frame, or None."""
        if not pose.header.frame_id or pose.header.frame_id == self.map_frame:
            return pose.pose.position.x, pose.pose.position.y
        m = self._lookup_matrix(self.map_frame, pose.header.frame_id, timeout=0.2)
        if m is None:
            return None
        x, y, _ = transform_xyz(m, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        return x, y

    # ------------------------------------------------------------------ waypoints
    def _load_gps_data(self):
        """Loads and parses the GPS file (GPX or YAML)."""
        if self.gps_file_name == "":
            self.get_logger().warn("No GPS file specified. GPS following will not be available.")
            return

        if os.path.isabs(self.gps_file_name):
            self.gps_path = self.gps_file_name
        else:
            candidates = [os.path.join(os.path.dirname(__file__), "..", "data", self.gps_file_name)]
            try:
                candidates.append(
                    os.path.join(
                        get_package_share_directory("robot_mission_planner"), "data", self.gps_file_name
                    )
                )
            except Exception:
                pass
            self.gps_path = next((c for c in candidates if os.path.exists(c)), candidates[0])

        if not os.path.exists(self.gps_path):
            self.get_logger().error(f"GPS file {self.gps_path} does not exist!")
            return

        try:
            if self.gps_path.endswith(".gpx"):
                with open(self.gps_path, "r") as f:
                    gpx = gpxpy.parse(f)
                points = list(gpx.waypoints)
                if not points:  # fall back to tracks / routes
                    points = [p for t in gpx.tracks for s in t.segments for p in s.points]
                if not points:
                    points = [p for r in gpx.routes for p in r.points]
                for wp in points:
                    self.waypoints_raw.append(
                        {"lat": wp.latitude, "lon": wp.longitude, "ele": wp.elevation or 0.0}
                    )
            elif self.gps_path.endswith((".yaml", ".yml")):
                with open(self.gps_path, "r") as f:
                    data = yaml.safe_load(f)
                for wp in data.get("waypoints", []):
                    self.waypoints_raw.append(
                        {"lat": wp["latitude"], "lon": wp["longitude"], "ele": wp.get("elevation", 0.0)}
                    )
            if self.reverse:
                self.waypoints_raw.reverse()
            self.get_logger().info(
                f"Loaded {len(self.waypoints_raw)} waypoints from {self.gps_path}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to parse GPS file: {e}")

    def _process_waypoints(self):
        """Convert raw lat/lon waypoints into backend goals and map_frame coordinates."""
        self.waypoints = [self._convert_to_msg(pt) for pt in self.waypoints_raw]
        self.waypoints_map = [self._raw_to_map(pt) for pt in self.waypoints_raw]
        self.number_waypoints = len(self.waypoints)

    def _raw_to_src(self, point):
        """Waypoint in the source frame (ECEF for commander, UTM for nav2)."""
        if self.nav_backend == "commander":
            return latlon_to_ecef(point["lat"], point["lon"], point.get("ele", 0.0))
        e, n, _, _ = utm.from_latlon(point["lat"], point["lon"])
        return e, n, point.get("ele", 0.0)

    def _raw_to_map(self, point):
        if self.src_to_map is None:
            return None
        x, y, _ = transform_xyz(self.src_to_map, *self._raw_to_src(point))
        return x, y

    def _convert_to_msg(self, point):
        """Backend goal message for one waypoint."""
        if self.nav_backend == "nav2" and not self.use_utm:
            msg = GeoPose()
            msg.position.latitude = point["lat"]
            msg.position.longitude = point["lon"]
            msg.position.altitude = point["ele"]
            return msg
        msg = PoseStamped()
        x, y, z = self._raw_to_src(point)
        if self.nav_backend == "commander":
            # The commander transforms poses into its map frame itself; publish in ECEF so
            # the waypoints stay valid if the local ENU origin changes between runs.
            msg.header.frame_id = self.earth_frame
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        else:
            msg.header.frame_id = self.map_frame
            mx, my, mz = transform_xyz(self.src_to_map, x, y, z)
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = mx, my, mz
        msg.pose.orientation.w = 1.0
        return msg

    def _waypoint_distance(self, idx, rob_xy):
        """Distance (m) from the robot to waypoint ``idx`` (inf if unknown)."""
        if self.waypoints_map and self.waypoints_map[idx] is not None and rob_xy is not None:
            return math.hypot(rob_xy[0] - self.waypoints_map[idx][0], rob_xy[1] - self.waypoints_map[idx][1])
        if self.pose_gps:  # lat/lon fallback (nav2 without UTM)
            target = self.waypoints_raw[idx]
            d_lat = (self.pose_gps["lat"] - target["lat"]) * 111320
            d_lon = (self.pose_gps["lon"] - target["lon"]) * 111320 * math.cos(math.radians(target["lat"]))
            return math.hypot(d_lat, d_lon)
        return float("inf")

    def _publish_waypoints_markers(self):
        if not self.waypoints_raw or not self._tf_ready or not self.waypoints_map:
            return
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, xy in enumerate(self.waypoints_map):
            if xy is None:
                continue
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = now
            marker.ns = "gps_waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y = xy
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 2.0
            marker.color.a = 0.8
            if i == self.current_waypoint_index:
                marker.color.g = 1.0
            else:
                marker.color.r = marker.color.g = 1.0
            marker_array.markers.append(marker)
        self._marker_pub.publish(marker_array)

    # ------------------------------------------------------------------ callbacks
    def _intersections_callback(self, msg):
        self._intersections = msg
        self._intersections_map = None  # re-transform lazily

    def _intersections_in_map(self):
        """Intersections as an (N, 2) array in map_frame, or None."""
        if self._intersections is None or not self._intersections.poses:
            return None
        if self._intersections_map is not None:
            return self._intersections_map
        pts = np.array([[p.position.x, p.position.y, p.position.z] for p in self._intersections.poses])
        frame = self._intersections.header.frame_id
        if frame and frame != self.map_frame:
            m = self._lookup_matrix(self.map_frame, frame, timeout=0.5)
            if m is None:
                return None
            pts = pts @ m[:3, :3].T + m[:3, 3]
        self._intersections_map = pts[:, :2]
        return self._intersections_map

    def _path_callback(self, msg):
        if not msg.poses:
            return
        self._latest_road_path = msg
        if self.state == self.STATE_ROAD and self._road_goal_needs_update(msg):
            self._send_road_goal()

    def _road_goal_needs_update(self, path_msg) -> bool:
        if not self._goal_active or self._last_road_goal is None:
            return True
        goal_xy = self._pose_to_map(path_msg.poses[-1])
        if goal_xy is None:
            return False
        moved = math.hypot(goal_xy[0] - self._last_road_goal[0], goal_xy[1] - self._last_road_goal[1])
        return moved > self.road_goal_update_distance

    def _commander_state_callback(self, msg):
        if msg.data != self.commander_mode:
            self.get_logger().info(f"Commander state: {msg.data}")
        self.commander_mode = msg.data

    def _gps_callback(self, msg):
        self.pose_gps = {"lat": msg.latitude, "lon": msg.longitude}
        if not self.path_send_url and (self.pose_ekf or not self.get_parameter("gps_filtered_topic").value):
            self.send_data_url("path")
            self.path_send_url = True

    def _ekf_callback(self, msg):
        self.pose_ekf = {"lat": msg.latitude, "lon": msg.longitude}
        if not self.path_send_url and self.pose_gps:
            self.send_data_url("path")
            self.path_send_url = True

    def send_data_url(self, msg_type):
        """Sends telemetry data to the remote server (disabled when telemetry_url is empty)."""
        if not self.telemetry_url:
            return
        data = self.data
        data["mission"] = {"current_waypoint_index": self.current_waypoint_index}
        if msg_type == "path":
            data["mission"]["waypoints"] = self.waypoints_raw
        if self.pose_gps:
            data["position"] = {"gps": self.pose_gps, "ekf": self.pose_ekf or self.pose_gps}
        else:
            data["position"] = {"gps": [], "ekf": []}
        try:
            response = requests.post(
                self.telemetry_url,
                headers={"Content-Type": "application/json"},
                data=json.dumps(data),
                timeout=1.0,
            )
            if response.status_code == 202 and msg_type != "path":
                self.send_data_url("path")
        except Exception:
            pass

    # ------------------------------------------------------------------ state machine
    def _main_logic_step(self):
        rob_xy = self._robot_xy()
        if rob_xy is None:
            return
        # Nav2 reports the waypoint index through action feedback; the commander does not.
        if self.state == self.STATE_ROAD or self.nav_backend == "commander":
            self._sync_waypoint_index_to_closest(rob_xy)
        if self.nav_backend == "commander" and self.state == self.STATE_ROAD:
            self._check_road_goal_reached(rob_xy)
        if self._intersections is not None:
            self._check_state_transitions(rob_xy)

    def _sync_waypoint_index_to_closest(self, rob_xy):
        """Move current_waypoint_index to the closest waypoint within a lookahead window."""
        if not self.waypoints:
            return
        num_wps = len(self.waypoints)
        best_idx, min_dist = self.current_waypoint_index, float("inf")
        if not self._waypoints_synced:
            # Initial sync: the robot may start anywhere along the route.
            candidates = range(self.start_index, num_wps)
        else:
            candidates = range(
                self.current_waypoint_index,
                self.current_waypoint_index + self.lookahead_sync_window,
            )
        for idx in candidates:
            if idx >= num_wps:
                if not self.loop:
                    break
                idx %= num_wps
            dist = self._waypoint_distance(idx, rob_xy)
            if dist < min_dist:
                min_dist, best_idx = dist, idx
        if math.isfinite(min_dist):
            self._waypoints_synced = True
        if best_idx != self.current_waypoint_index:
            self.get_logger().info(
                f"Waypoint sync: moving index {self.current_waypoint_index} -> {best_idx} "
                f"(dist {min_dist:.2f} m)"
            )
            self.current_waypoint_index = best_idx
        if min_dist < self.gps_threshold:
            next_idx = self.current_waypoint_index + 1
            if next_idx >= num_wps:
                next_idx = 0 if self.loop else num_wps - 1
            if next_idx != self.current_waypoint_index:
                self.get_logger().info(
                    f"Reached waypoint {self.current_waypoint_index} ({min_dist:.2f} m); "
                    f"advancing to {next_idx}"
                )
                self.current_waypoint_index = next_idx

    def _check_road_goal_reached(self, rob_xy):
        """Commander backend: clear the active road goal once the robot is close to it."""
        if not self._goal_active or self._last_road_goal is None:
            return
        d = math.hypot(rob_xy[0] - self._last_road_goal[0], rob_xy[1] - self._last_road_goal[1])
        if d < self.gps_threshold:
            self.get_logger().info(f"Road goal reached ({d:.2f} m); next path update re-sends.")
            self._goal_active = False

    def _check_state_transitions(self, rob_xy):
        inter = self._intersections_in_map()
        if self.state == self.STATE_ROAD:
            if inter is None:
                return
            d = np.hypot(inter[:, 0] - rob_xy[0], inter[:, 1] - rob_xy[1])
            i = int(np.argmin(d))
            if d[i] < self.enter_threshold:
                self.get_logger().info(
                    f"Approaching intersection ({d[i]:.2f} m). Switching to GPS mode."
                )
                self.state = self.STATE_GPS
                self._active_intersection = tuple(inter[i])
                self._cancel_current_goal()
                self._schedule_goal(delay_sec=1.0, mode="GPS")
            return

        # STATE_GPS: leave once clear of *all* nearby intersections
        if inter is not None:
            closest = float(np.min(np.hypot(inter[:, 0] - rob_xy[0], inter[:, 1] - rob_xy[1])))
        elif self._active_intersection is not None:
            closest = math.hypot(rob_xy[0] - self._active_intersection[0], rob_xy[1] - self._active_intersection[1])
        else:
            return
        if closest <= self.exit_threshold:
            return
        if self.require_wp_to_exit and self.waypoints:
            if self._waypoint_distance(self.current_waypoint_index, rob_xy) >= self.gps_threshold:
                return
        self.get_logger().info(
            f"Passed nearby intersections (closest {closest:.2f} m). Switching back to ROAD mode."
        )
        self.state = self.STATE_ROAD
        self._active_intersection = None
        self._last_road_goal = None
        self._cancel_current_goal()
        self._schedule_goal(delay_sec=1.0, mode="ROAD")

    # ------------------------------------------------------------------ goal dispatch
    def _schedule_goal(self, delay_sec, mode):
        if self._pending_goal_timer:
            self._pending_goal_timer.cancel()
        cb = self._send_gps_goal_timer_cb if mode == "GPS" else self._send_road_goal_timer_cb
        self._pending_goal_timer = self.create_timer(delay_sec, cb)

    def _send_road_goal_timer_cb(self):
        self._pending_goal_timer.cancel()
        self._pending_goal_timer = None
        self._send_road_goal()

    def _send_gps_goal_timer_cb(self):
        self._pending_goal_timer.cancel()
        self._pending_goal_timer = None
        self._send_gps_goal()

    def _cancel_current_goal(self):
        if self.nav_backend == "commander":
            self._commander_switch_mode("stop")
        elif self._goal_handle is not None:
            self.get_logger().info("Cancelling current goal for state transition.")
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._goal_active = False

    def _send_road_goal(self):
        if (
            self.state != self.STATE_ROAD
            or self._latest_road_path is None
            or not self._latest_road_path.poses
        ):
            return
        pose_stamped = self._latest_road_path.poses[-1]
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        goal_xy = self._pose_to_map(pose_stamped)
        if goal_xy is None:
            self.get_logger().warn("Road goal frame not transformable to map_frame yet; skipping.")
            return
        self.get_logger().info(
            f"Road goal: ({goal_xy[0]:.2f}, {goal_xy[1]:.2f}) in {self.map_frame} "
            f"(from {pose_stamped.header.frame_id})"
        )
        self._goal_active = True
        self._threshold_triggered = False
        self._last_road_goal = goal_xy

        if self.nav_backend == "commander":
            self._commander_switch_mode("goto")
            self._pub_goal_waypoint.publish(pose_stamped)
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        self.send_goal_future = self._road_action_client.send_goal_async(
            goal_msg, feedback_callback=self._road_feedback_callback
        )
        self.send_goal_future.add_done_callback(self._goal_response_callback)

    def _send_gps_goal(self):
        if self.state != self.STATE_GPS or not self.waypoints:
            return
        rob_xy = self._robot_xy()
        if rob_xy is not None:
            self._sync_waypoint_index_to_closest(rob_xy)
        remaining = self.waypoints[self.current_waypoint_index :]
        if not remaining:
            self.get_logger().warn("No remaining GPS waypoints to send.")
            return
        self._gps_start_index = self.current_waypoint_index
        self.get_logger().info(
            f"GPS goal: sending {len(remaining)} waypoints from index {self.current_waypoint_index}."
        )
        self._goal_active = True

        if self.nav_backend == "commander":
            seq = PoseArray()
            seq.header.frame_id = remaining[0].header.frame_id
            seq.header.stamp = self.get_clock().now().to_msg()
            seq.poses = [wp.pose for wp in remaining]
            self._commander_configure_sequence(lambda: self._publish_sequence(seq))
            return
        if not self.use_utm:
            goal_msg = FollowGPSWaypoints.Goal()
            goal_msg.gps_poses = remaining
        else:
            goal_msg = FollowWaypoints.Goal()
            goal_msg.poses = remaining
        self.send_goal_future = self._gps_action_client.send_goal_async(
            goal_msg, feedback_callback=self._gps_feedback_callback
        )
        self.send_goal_future.add_done_callback(self._goal_response_callback)

    # ------------------------------------------------------------------ commander backend
    def _publish_sequence(self, seq: PoseArray):
        self._pub_goal_sequence.publish(seq)
        self._commander_switch_mode("sequence")

    def _commander_configure_sequence(self, then):
        """Make the commander take its sequence from the topic, then call ``then``."""
        if self._sequence_configured:
            then()
            return
        cli = self._cli_configure_seq
        if not cli.service_is_ready():
            self.get_logger().warn(
                f"{cli.srv_name} not ready; publishing the sequence anyway (commander must be "
                "configured with sequence_source=topic)."
            )
            then()
            return
        req = self._srv_types["configure"].Request()
        req.source = req.SOURCE_TOPIC
        req.gpx_file_name = ""
        req.loop = bool(self.loop)

        def done(fut):
            try:
                res = fut.result()
                self.get_logger().info(f"configure_sequence_mode: {res.success} {res.message}")
                self._sequence_configured = bool(res.success)
            except Exception as e:
                self.get_logger().error(f"configure_sequence_mode failed: {e}")
            then()

        cli.call_async(req).add_done_callback(done)

    def _commander_switch_mode(self, mode: str):
        # The state topic lags the request; remember what we asked for so that a burst of
        # path messages does not turn into a burst of identical service calls.
        if self._requested_mode == mode:
            return
        if self.commander_mode is not None and self.commander_mode.lower() == mode:
            self._requested_mode = mode
            return
        cli = self._cli_switch_mode
        if not cli.service_is_ready():
            self.get_logger().warn(f"{cli.srv_name} not ready; cannot switch to '{mode}'.")
            return
        self._requested_mode = mode
        req = self._srv_types["switch"].Request()
        req.mode = mode

        def done(fut):
            try:
                res = fut.result()
                level = self.get_logger().info if res.success else self.get_logger().error
                level(f"switch_mode('{mode}'): {res.success} {res.message}")
                if not res.success and self._requested_mode == mode:
                    self._requested_mode = None  # allow a retry
            except Exception as e:
                self.get_logger().error(f"switch_mode('{mode}') failed: {e}")
                if self._requested_mode == mode:
                    self._requested_mode = None

        cli.call_async(req).add_done_callback(done)

    # ------------------------------------------------------------------ nav2 backend
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
        dist = feedback_msg.feedback.distance_remaining
        if dist == 0.0:
            return
        if dist < self.gps_threshold and not self._threshold_triggered:
            self._threshold_triggered = True
            self.get_logger().info(f"Road distance threshold reached ({dist:.2f} m).")
            self._goal_active = False

    def _gps_feedback_callback(self, feedback_msg):
        new_global_idx = self._gps_start_index + feedback_msg.feedback.current_waypoint
        if new_global_idx != self.current_waypoint_index:
            self.get_logger().info(f"GPS feedback: reached waypoint {new_global_idx}")
            self.current_waypoint_index = new_global_idx

    def _result_callback(self, future):
        status = future.result().status
        self.get_logger().info(f"Goal finished with status: {status}")
        self._goal_active = False
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.state == self.STATE_GPS and self.loop:
                self.current_waypoint_index = 0
                self._send_gps_goal()
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("Goal aborted.")

    # ------------------------------------------------------------------ shutdown
    def save_waypoint_index(self):
        if not self.gps_path:
            return
        try:
            index_dir = os.path.join(os.path.dirname(self.gps_path), "waypoint_index")
            os.makedirs(index_dir, exist_ok=True)
            path = os.path.join(index_dir, f"{int(time.time())}.txt")
            with open(path, "w") as f:
                f.write(str(self.current_waypoint_index))
            self.get_logger().info(f"Saved current waypoint index {self.current_waypoint_index} to {path}")
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
