#!/usr/bin/env python3
"""
Road follower with GPS fallback at intersections.

State machine
-------------
ROAD  : follow the visually detected road. The goal is taken from ``road_goal_source``:
        ``carrot`` (default) drives at the convex-hull centre of the road points in the
        current lidar frame (``carrot_topic``, a visualization_msgs/Marker from
        build_point_cloud, or a nav_msgs/Path whose last pose is used); ``path`` takes
        the predicted road path (``road_points_topic``, nav_msgs/Path from
        path_centerline). Either way the goal is kept at least ``road_goal_min_ahead``
        in front of the robot (see ``road_goal.py``), because the commander treats a
        goal inside its 2.5 m arrival box as already reached and stops.
GPS   : follow the pre-planned GPX waypoints instead. Entered near an OSM intersection
        (``intersections_topic``, geometry_msgs/PoseArray from map_data/osm_cloud), when
        the road path stops arriving (``road_path_timeout``) or when the commander reports
        being stuck (``stuck_fallback_to_gps``).

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
from geometry_msgs.msg import Point, PoseArray, PoseStamped
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

from robot_mission_planner.road_goal import (
    is_behind,
    select_carrot_goal,
    select_path_goal,
    smooth,
)

# WGS84 (lat/lon -> ECEF for the commander backend; kept local to avoid a map_data dependency)
_WGS84_A = 6378137.0
_WGS84_E2 = (1.0 / 298.257223563) * (2.0 - 1.0 / 298.257223563)

GPS_REASON_INTERSECTION = "intersection"
GPS_REASON_NO_ROAD = "no_road"
GPS_REASON_STUCK = "stuck"


def latlon_to_ecef(lat_deg: float, lon_deg: float, alt_m: float = 0.0) -> tuple[float, float, float]:
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * math.sin(lat) ** 2)
    x = (n + alt_m) * math.cos(lat) * math.cos(lon)
    y = (n + alt_m) * math.cos(lat) * math.sin(lon)
    z = (n * (1.0 - _WGS84_E2) + alt_m) * math.sin(lat)
    return x, y, z


def marker_point_in_header_frame(marker: Marker, point):
    """A Marker ``points[]`` entry expressed in ``marker.header.frame_id`` (they are relative to ``marker.pose``)."""
    m = numpify(marker.pose)
    x, y, z = transform_xyz(m, point.x, point.y, point.z)
    return Point(x=float(x), y=float(y), z=float(z))


def transform_xyz(matrix: np.ndarray, x: float, y: float, z: float = 0.0) -> tuple[float, float, float]:
    """Apply a 4x4 homogeneous matrix to a point."""
    p = matrix[:3, :3] @ np.array([x, y, z]) + matrix[:3, 3]
    return float(p[0]), float(p[1]), float(p[2])


def distance_to_polyline(point, segments_a: np.ndarray, segments_b: np.ndarray) -> float:
    """Minimum distance from ``point`` (x, y) to the polyline given as segment endpoints."""
    if segments_a.size == 0:
        return float("inf")
    p = np.asarray(point, dtype=float)
    ab = segments_b - segments_a
    ap = p - segments_a
    denom = np.einsum("ij,ij->i", ab, ab)
    t = np.where(denom > 0, np.einsum("ij,ij->i", ap, ab) / np.where(denom > 0, denom, 1.0), 0.0)
    t = np.clip(t, 0.0, 1.0)
    closest = segments_a + ab * t[:, None]
    return float(np.min(np.hypot(*(p - closest).T)))


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
        self.declare_parameter("commander_state_topic", "/crl_commander/state")
        self.declare_parameter("state_topic", "~/state")  # latched String: ROAD | GPS:<reason>
        # latched PoseStamped in map_frame of the intersection that triggered GPS mode
        # (empty frame_id = none); the map_data viewer draws the enter/exit circles around it
        self.declare_parameter("active_intersection_topic", "~/active_intersection")
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
        self.declare_parameter("telemetry_url", "")  # empty = no telemetry POSTs

        # --- Thresholds ---
        self.declare_parameter("intersection_enter_threshold", 3.0)  # m: ROAD -> GPS
        self.declare_parameter("intersection_exit_threshold", 4.0)  # m: from all intersections
        self.declare_parameter("gps_goal_threshold", 3.0)  # m: waypoint reached
        self.declare_parameter("lookahead_sync_window", 15)  # waypoints searched for the closest
        self.declare_parameter("road_goal_update_distance", 1.0)  # m: re-send active road goal
        # GPS -> ROAD hysteresis: require having passed the intersection along the route
        # direction (not just a radius), and/or a number of waypoints advanced since entry.
        self.declare_parameter("gps_exit_require_passed", True)
        self.declare_parameter("gps_exit_min_waypoints", 0)
        # Legacy: additionally require being within gps_goal_threshold of the current waypoint.
        self.declare_parameter("require_waypoint_reached_to_exit_gps", False)
        # Road-goal sanity: reject goals farther than this from the planned route (0 = off)
        # or behind the robot, so a bad segmentation cannot pull us off the mission.
        self.declare_parameter("road_goal_max_route_offset", 5.0)
        self.declare_parameter("road_goal_reject_behind", True)
        # Where the ROAD goal comes from: "carrot" = one road-centre point per lidar frame
        # (convex-hull centre from build_point_cloud), "path" = the fitted/extrapolated
        # /predicted_path_ls from path_predictor.
        self.declare_parameter("road_goal_source", "carrot")
        self.declare_parameter("carrot_topic", "/cloud_hull_center_marker")
        self.declare_parameter("carrot_type", "marker")  # marker | path (last pose)
        # The goal is never closer than min_ahead (commander arrival box is 2.5 m) and a
        # candidate farther than max_ahead is discarded as a projection artefact.
        self.declare_parameter("road_goal_min_ahead", 4.0)
        self.declare_parameter("road_goal_max_ahead", 12.0)
        self.declare_parameter("road_goal_smoothing", 0.0)  # carrot: 0 = raw, 0.8 = damped
        # Commander backend: forget the active road goal once this close to it, so the
        # next observation re-sends one (the commander's own arrival box is 2.5 m).
        self.declare_parameter("road_goal_reached_distance", 2.5)
        # Failure handling
        self.declare_parameter("road_path_timeout", 5.0)  # s without a road path -> GPS (0 = off)
        self.declare_parameter("stuck_fallback_to_gps", True)  # commander STUCK in ROAD -> GPS
        self.declare_parameter("service_timeout", 3.0)  # s: commander service call watchdog
        self.declare_parameter("gps_sequence_window", 10)  # waypoints per sequence (0 = all)

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
        self.gps_exit_require_passed = gp("gps_exit_require_passed")
        self.gps_exit_min_waypoints = gp("gps_exit_min_waypoints")
        self.require_wp_to_exit = gp("require_waypoint_reached_to_exit_gps")
        self.road_goal_max_route_offset = gp("road_goal_max_route_offset")
        self.road_goal_reject_behind = gp("road_goal_reject_behind")
        self.road_goal_source = gp("road_goal_source")
        if self.road_goal_source not in ("carrot", "path"):
            self.get_logger().error(
                f"Unknown road_goal_source '{self.road_goal_source}', using 'carrot'"
            )
            self.road_goal_source = "carrot"
        self.carrot_type = gp("carrot_type")
        self.road_goal_min_ahead = gp("road_goal_min_ahead")
        self.road_goal_max_ahead = gp("road_goal_max_ahead")
        self.road_goal_smoothing = gp("road_goal_smoothing")
        self.road_goal_reached_distance = gp("road_goal_reached_distance")
        self.road_path_timeout = gp("road_path_timeout")
        self.stuck_fallback_to_gps = gp("stuck_fallback_to_gps")
        self.service_timeout = gp("service_timeout")
        self.gps_sequence_window = gp("gps_sequence_window")

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
        self._state_pub = self.create_publisher(String, gp("state_topic"), latched)
        self._active_int_pub = self.create_publisher(
            PoseStamped, gp("active_intersection_topic"), latched
        )
        self._published_active = object()  # sentinel so the first state is always published
        self.create_timer(5.0, self._publish_waypoints_markers)

        # --- Waypoints ---
        self.current_waypoint_index = self.start_index
        self.number_waypoints = 0
        self.waypoints_raw = []  # dicts lat/lon/ele
        self.waypoints = []  # backend goal messages (PoseStamped in src frame, or GeoPose)
        self.waypoints_map = []  # (x, y) in map_frame for distance checks
        self._route_a = np.empty((0, 2))  # route polyline segments in map_frame
        self._route_b = np.empty((0, 2))
        self.gps_path = ""
        self._load_gps_data()

        if self.nav_backend == "nav2" and not self.use_utm:
            self._tf_ready = True  # lat/lon goals, no transform needed
            self._process_waypoints()
        else:
            self._utm_timer = self.create_timer(1.0, self._resolve_waypoint_transform)

        # --- Runtime state ---
        self._latest_road_path = None  # nav_msgs/Path (road_goal_source=path)
        self._latest_carrot = None  # (x, y) in map_frame (road_goal_source=carrot)
        self._last_road_path_time = None  # node clock seconds of the last road path
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
        self._requested_mode_time = 0.0
        self._mode_settle_time = 2.0  # s after a switch before the state topic is trusted
        self._waypoints_synced = False  # first sync searches the whole list
        self._gps_reason = None  # why GPS mode was entered
        self._gps_entry_index = 0  # waypoint index when GPS mode was entered
        self._gps_route_dir = None  # unit route direction at the active intersection
        self._service_watchdogs = []

        self.pose_gps = None
        self.pose_ekf = None
        self.path_send_url = False
        self.data = {"robot_id": self.robot_id}

        # --- Subscriptions ---
        if self.road_goal_source == "path":
            self.create_subscription(Path, gp("road_points_topic"), self._path_callback, 10)
        else:
            carrot_msg = Path if self.carrot_type == "path" else Marker
            self.create_subscription(carrot_msg, gp("carrot_topic"), self._carrot_callback, 10)
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
            f"Road goal: {self.road_goal_source} "
            f"({gp('carrot_topic') if self.road_goal_source == 'carrot' else gp('road_points_topic')}), "
            f"ahead {self.road_goal_min_ahead}-{self.road_goal_max_ahead} m; "
            f"intersections: {gp('intersections_topic')}, "
            f"GPS file: {self.gps_file_name}\n"
            f"Thresholds: enter={self.enter_threshold} m, exit={self.exit_threshold} m, "
            f"waypoint={self.gps_threshold} m, route offset={self.road_goal_max_route_offset} m, "
            f"road path timeout={self.road_path_timeout} s"
        )
        self._start_time = self._now()
        self.create_timer(1.0, self._main_logic_step)

    # ------------------------------------------------------------------ TF helpers
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

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

    def _robot_pose(self):
        """Robot (x, y, yaw) in map_frame, or None."""
        m = self._lookup_matrix(self.map_frame, self.robot_frame, timeout=0.2)
        if m is None:
            return None
        return float(m[0, 3]), float(m[1, 3]), float(math.atan2(m[1, 0], m[0, 0]))

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
        xy = np.array([p for p in self.waypoints_map if p is not None], dtype=float)
        if len(xy) >= 2:
            self._route_a, self._route_b = xy[:-1], xy[1:]
        else:
            self._route_a = self._route_b = np.empty((0, 2))

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

    def _route_direction_at(self, idx):
        """Unit vector of the route around waypoint ``idx`` in map_frame, or None."""
        if not self.waypoints_map:
            return None
        n = len(self.waypoints_map)
        i0, i1 = max(0, min(idx, n - 1)), min(n - 1, idx + 1)
        if i0 == i1:
            i0 = max(0, i1 - 1)
        a, b = self.waypoints_map[i0], self.waypoints_map[i1]
        if a is None or b is None:
            return None
        d = np.array([b[0] - a[0], b[1] - a[1]])
        norm = np.linalg.norm(d)
        return d / norm if norm > 1e-6 else None

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
        self._road_input()

    def _carrot_callback(self, msg):
        """One road-centre point (Marker) or the last pose of a Path -> (x, y) in map_frame."""
        if isinstance(msg, Path):
            if not msg.poses:
                return
            src = msg.poses[-1]
        else:
            # build_point_cloud's per-frame /cloud_hull_center_marker is a SPHERE with the
            # centre in pose.position; build_map's accumulated /map_hull_center_marker is a
            # SPHERE_LIST whose newest centre is the last of points[] (pose is identity).
            src = PoseStamped()
            src.header = msg.header
            src.pose = msg.pose
            if msg.points:
                src.pose.position = marker_point_in_header_frame(msg, msg.points[-1])
        xy = self._pose_to_map(src)
        if xy is None:
            return
        self._latest_carrot = smooth(self._latest_carrot, xy, self.road_goal_smoothing)
        self._road_input()

    def _road_input(self):
        """A new road observation arrived: refresh the ROAD goal when it moved enough."""
        self._last_road_path_time = self._now()
        if self.state != self.STATE_ROAD:
            return
        candidate = self._road_goal_candidate()
        if candidate is not None and self._road_goal_needs_update(candidate[:2]):
            self._send_road_goal(candidate)

    def _road_goal_candidate(self):
        """(x, y, yaw) of the next ROAD goal in map_frame, or None."""
        pose = self._robot_pose()
        if pose is None:
            return None
        rob_xy, yaw = pose[:2], pose[2]
        if self.road_goal_source == "carrot":
            if self._latest_carrot is None:
                return None
            return select_carrot_goal(
                self._latest_carrot, rob_xy, yaw, self.road_goal_min_ahead, self.road_goal_max_ahead
            )
        if self._latest_road_path is None or not self._latest_road_path.poses:
            return None
        pts = [self._pose_to_map(p) for p in self._latest_road_path.poses]
        pts = [p for p in pts if p is not None]
        return select_path_goal(pts, rob_xy, yaw, self.road_goal_min_ahead, self.road_goal_max_ahead)

    def _road_goal_needs_update(self, goal_xy) -> bool:
        if not self._goal_active or self._last_road_goal is None:
            return True
        moved = math.hypot(goal_xy[0] - self._last_road_goal[0], goal_xy[1] - self._last_road_goal[1])
        return moved > self.road_goal_update_distance

    def _road_path_fresh(self) -> bool:
        if self.road_path_timeout <= 0:
            return True
        if self._last_road_path_time is None:
            return False
        return (self._now() - self._last_road_path_time) < self.road_path_timeout

    def _commander_state_callback(self, msg):
        if msg.data != self.commander_mode:
            self.get_logger().info(f"Commander state: {msg.data}")
        self.commander_mode = msg.data
        # The commander leaves our mode on its own (sequence finished -> STOP, operator
        # intervention, STUCK). Forget the request so the next switch is actually sent.
        if (
            self._requested_mode is not None
            and msg.data.lower() != self._requested_mode
            and (self._now() - self._requested_mode_time) > self._mode_settle_time
        ):
            self._requested_mode = None

    def _commander_left_us(self) -> bool:
        """True once the commander sits in STOP although we asked for goto/sequence."""
        return (
            self.commander_mode is not None
            and self.commander_mode.upper() == "STOP"
            and self._requested_mode is None
            and self._goal_active
        )

    def _commander_stuck(self) -> bool:
        return self.commander_mode is not None and "STUCK" in self.commander_mode.upper()

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
    def _publish_state(self):
        text = "ROAD" if self.state == self.STATE_ROAD else f"GPS:{self._gps_reason}"
        self._state_pub.publish(String(data=text))
        active = self._active_intersection if self.state == self.STATE_GPS else None
        if active != self._published_active:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            if active is not None:
                msg.header.frame_id = self.map_frame
                msg.pose.position.x, msg.pose.position.y = float(active[0]), float(active[1])
            msg.pose.orientation.w = 1.0
            self._active_int_pub.publish(msg)
            self._published_active = active

    def _main_logic_step(self):
        self._publish_state()
        pose = self._robot_pose()
        if pose is None:
            return
        rob_xy = pose[:2]
        # Nav2 reports the waypoint index through action feedback; the commander does not.
        if self.state == self.STATE_ROAD or self.nav_backend == "commander":
            self._sync_waypoint_index_to_closest(rob_xy)
        if self.nav_backend == "commander" and self.state == self.STATE_ROAD:
            self._check_road_goal_reached(rob_xy)
        if self.nav_backend == "commander" and self._commander_left_us():
            if self.state == self.STATE_GPS:
                # The sequence window (gps_sequence_window) was consumed: send the next one.
                self.get_logger().info("Commander finished the sequence window; sending the next one.")
                self._send_gps_goal()
            else:
                self.get_logger().info("Commander stopped during ROAD; re-sending the road goal.")
                self._goal_active = False
                self._send_road_goal()
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
        if d < self.road_goal_reached_distance:
            self.get_logger().info(
                f"Road goal reached ({d:.2f} m); the next road observation re-sends.",
                throttle_duration_sec=2.0,
            )
            self._goal_active = False

    def _closest_intersection(self, rob_xy):
        """(distance, (x, y)) of the closest intersection in map_frame, or (inf, None)."""
        inter = self._intersections_in_map()
        if inter is not None:
            d = np.hypot(inter[:, 0] - rob_xy[0], inter[:, 1] - rob_xy[1])
            i = int(np.argmin(d))
            return float(d[i]), (float(inter[i, 0]), float(inter[i, 1]))
        if self._active_intersection is not None:
            return (
                math.hypot(rob_xy[0] - self._active_intersection[0], rob_xy[1] - self._active_intersection[1]),
                self._active_intersection,
            )
        return float("inf"), None

    def _check_state_transitions(self, rob_xy):
        closest, closest_xy = self._closest_intersection(rob_xy)

        if self.state == self.STATE_ROAD:
            if closest < self.enter_threshold:
                self._enter_gps(GPS_REASON_INTERSECTION, closest_xy, f"approaching intersection ({closest:.2f} m)")
            elif self.stuck_fallback_to_gps and self._commander_stuck():
                self._enter_gps(GPS_REASON_STUCK, None, "commander reports STUCK")
            elif not self._road_path_fresh() and (self._now() - self._start_time) > self.road_path_timeout:
                self._enter_gps(GPS_REASON_NO_ROAD, None, f"no road path for {self.road_path_timeout} s")
            return

        # STATE_GPS: decide whether to go back to road following
        if closest < self.enter_threshold and self._gps_reason != GPS_REASON_INTERSECTION:
            # A fallback GPS run reached an intersection: treat it as an intersection entry.
            self._gps_reason = GPS_REASON_INTERSECTION
            self._active_intersection = closest_xy
            self._gps_entry_index = self.current_waypoint_index
            self._gps_route_dir = self._route_direction_at(self.current_waypoint_index)
            return

        advanced = self.current_waypoint_index - self._gps_entry_index
        if advanced < self.gps_exit_min_waypoints:
            return

        if self._gps_reason == GPS_REASON_INTERSECTION:
            if closest <= self.exit_threshold:
                return
            if self.gps_exit_require_passed and self._active_intersection is not None and self._gps_route_dir is not None:
                rel = np.array([rob_xy[0] - self._active_intersection[0], rob_xy[1] - self._active_intersection[1]])
                if float(rel @ self._gps_route_dir) <= 0.0:
                    return  # still before the intersection along the route
            if self.require_wp_to_exit and self.waypoints:
                if self._waypoint_distance(self.current_waypoint_index, rob_xy) >= self.gps_threshold:
                    return
            why = f"passed intersection (closest {closest:.2f} m)"
        elif self._gps_reason == GPS_REASON_STUCK:
            if self._commander_stuck() or advanced < max(1, self.gps_exit_min_waypoints):
                return
            why = "commander no longer stuck"
        else:  # GPS_REASON_NO_ROAD
            if not self._road_path_fresh() or closest <= self.exit_threshold:
                return
            why = "road path available again"

        self._enter_road(why)

    def _enter_gps(self, reason, intersection_xy, why):
        self.get_logger().info(f"{why}. Switching to GPS mode ({reason}).")
        self.state = self.STATE_GPS
        self._gps_reason = reason
        self._active_intersection = intersection_xy
        self._gps_entry_index = self.current_waypoint_index
        self._gps_route_dir = self._route_direction_at(self.current_waypoint_index)
        self._cancel_current_goal()
        self._schedule_goal(delay_sec=1.0, mode="GPS")

    def _enter_road(self, why):
        self.get_logger().info(f"{why}. Switching back to ROAD mode.")
        self.state = self.STATE_ROAD
        self._gps_reason = None
        self._active_intersection = None
        self._gps_route_dir = None
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

    def _road_goal_valid(self, goal_xy) -> bool:
        """Sanity-check a road goal against the planned route and the robot heading."""
        if self.road_goal_max_route_offset > 0 and self._route_a.size:
            off = distance_to_polyline(goal_xy, self._route_a, self._route_b)
            if off > self.road_goal_max_route_offset:
                self.get_logger().warn(
                    f"Road goal rejected: {off:.1f} m off the planned route "
                    f"(> {self.road_goal_max_route_offset} m)",
                    throttle_duration_sec=2.0,
                )
                return False
        if self.road_goal_reject_behind:
            pose = self._robot_pose()
            if pose is not None and is_behind(goal_xy, pose[:2], pose[2]):
                self.get_logger().warn("Road goal rejected: behind the robot", throttle_duration_sec=2.0)
                return False
        return True

    def _send_road_goal(self, candidate=None):
        if self.state != self.STATE_ROAD:
            return
        if candidate is None:
            candidate = self._road_goal_candidate()
        if candidate is None:
            return
        goal_xy = candidate[:2]
        if not self._road_goal_valid(goal_xy):
            return
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self.map_frame
        pose_stamped.pose.position.x, pose_stamped.pose.position.y = float(goal_xy[0]), float(goal_xy[1])
        pose_stamped.pose.orientation.z = math.sin(candidate[2] / 2.0)
        pose_stamped.pose.orientation.w = math.cos(candidate[2] / 2.0)
        self.get_logger().info(
            f"Road goal ({self.road_goal_source}): ({goal_xy[0]:.2f}, {goal_xy[1]:.2f}) "
            f"in {self.map_frame}, yaw {math.degrees(candidate[2]):.0f} deg",
            throttle_duration_sec=2.0,
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
        pose = self._robot_pose()
        if pose is not None:
            self._sync_waypoint_index_to_closest(pose[:2])
        start = self.current_waypoint_index
        end = start + self.gps_sequence_window if self.gps_sequence_window > 0 else len(self.waypoints)
        remaining = self.waypoints[start:end]
        if not remaining:
            self.get_logger().warn("No remaining GPS waypoints to send (end of mission).")
            self._goal_active = False
            return
        self._gps_start_index = start
        self.get_logger().info(f"GPS goal: sending {len(remaining)} waypoints from index {start}.")
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
    def _watch_service_call(self, future, what, on_timeout=None):
        """Log (and optionally react) when a service call does not return in time."""
        if self.service_timeout <= 0:
            return

        def check():
            timer.cancel()
            self._service_watchdogs = [t for t in self._service_watchdogs if t is not timer]
            if not future.done():
                self.get_logger().error(f"{what} did not respond within {self.service_timeout} s")
                if on_timeout:
                    on_timeout()

        timer = self.create_timer(self.service_timeout, check)
        self._service_watchdogs.append(timer)

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
        called = {"then": False}

        def run_then():
            if not called["then"]:
                called["then"] = True
                then()

        def done(fut):
            try:
                res = fut.result()
                self.get_logger().info(f"configure_sequence_mode: {res.success} {res.message}")
                self._sequence_configured = bool(res.success)
            except Exception as e:
                self.get_logger().error(f"configure_sequence_mode failed: {e}")
            run_then()

        future = cli.call_async(req)
        future.add_done_callback(done)
        self._watch_service_call(future, "configure_sequence_mode", on_timeout=run_then)

    def _commander_switch_mode(self, mode: str):
        # The state topic lags the request; remember what we asked for so that a burst of
        # path messages does not turn into a burst of identical service calls.
        if self._requested_mode == mode:
            return
        if self.commander_mode is not None and self.commander_mode.lower() == mode:
            self._requested_mode = mode
            self._requested_mode_time = self._now()
            return
        cli = self._cli_switch_mode
        if not cli.service_is_ready():
            self.get_logger().warn(f"{cli.srv_name} not ready; cannot switch to '{mode}'.")
            return
        self._requested_mode = mode
        self._requested_mode_time = self._now()
        req = self._srv_types["switch"].Request()
        req.mode = mode

        def reset_request():
            if self._requested_mode == mode:
                self._requested_mode = None  # allow a retry

        def done(fut):
            try:
                res = fut.result()
                level = self.get_logger().info if res.success else self.get_logger().error
                level(f"switch_mode('{mode}'): {res.success} {res.message}")
                if not res.success:
                    reset_request()
            except Exception as e:
                self.get_logger().error(f"switch_mode('{mode}') failed: {e}")
                reset_request()

        future = cli.call_async(req)
        future.add_done_callback(done)
        self._watch_service_call(future, f"switch_mode('{mode}')", on_timeout=reset_request)

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
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.save_waypoint_index()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
