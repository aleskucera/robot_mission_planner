#!/usr/bin/env python3
"""
The follower: it drives the robot along a road, along a route, or both.

Modes (``mode``, see ``follower/modes.py``)
------------------------------------------
road_gps : follow the detected road and hand over to the route's waypoints around OSM
           intersections, when the road detection drops out, when the commander reports
           being stuck, and for the final metres to the goal. The Robotour mode.
gps      : follow the route's waypoints from beginning to end, never look at the road.
road     : follow the road only -- no route, no intersections, no goal to arrive at.

The route of the two route modes is either a GPX/YAML file (``file``) or planned by
route_planner from a mission goal (a QR code); that is a separate choice, not a mode.

State machine
-------------
ROAD  : follow the visually detected road. The goal is taken from ``road_goal_source``:
        ``carrot`` (default) drives at the convex-hull centre of the road points in the
        current lidar frame (``carrot_topic``, a visualization_msgs/Marker from
        build_point_cloud, or a nav_msgs/Path whose last pose is used); ``path`` takes
        the predicted road path (``road_points_topic``, nav_msgs/Path from
        path_centerline); ``route`` stretches the carrot along the planned OSM route
        (``route_stretch_distance`` further along it, keeping the carrot's own lateral
        offset from the route), so the goal follows the road's mapped shape while the
        map is only ever used relative to the robot's own projection on it. Either way
        the goal is kept at least ``road_goal_min_ahead`` in front of the robot (see
        ``road_goal.py``), because the commander treats a goal inside its 2.5 m arrival
        box as already reached and stops.
GPS   : follow the pre-planned GPX waypoints instead. Entered near an OSM intersection
        (``intersections_topic``, geometry_msgs/PoseArray from map_data/osm_cloud), when
        the road path stops arriving (``road_path_timeout``) or when the commander reports
        being stuck (``stuck_fallback_to_gps``).

Navigation backends (``nav_backend``, see ``follower/backends/``)
----------------------------------------------------------------
commander   : the Helhest field stack (crl_commander on the NUC). ROAD goals are published
              as a PoseStamped on ``goal_waypoint_topic`` in *goto* mode; GPS waypoints are
              published as a latched PoseArray on ``goal_sequence_topic`` (in ``earth_frame``,
              ECEF) and the commander is switched to *sequence* mode.
nav2        : Nav2 ``NavigateToPose`` / ``FollowWaypoints`` (``FollowGPSWaypoints`` when
              ``use_utm`` is false).
follow_path : a bare pure-pursuit controller (``path_follower``): each goal becomes a short
              path from the robot to it. No waypoint sequence, so ``mode: road`` only.

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

import numpy as np
import rclpy
import requests
from ament_index_python.packages import get_package_share_directory
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from robot_mission_planner.follower import modes
from robot_mission_planner.follower.backends import KINDS as BACKEND_KINDS, make_backend
from robot_mission_planner.follower.frames import (
    Frames,
    distance_to_polyline,
    marker_point_in_header_frame,
)
from robot_mission_planner.follower.road_goal import (
    indices_near_polyline,
    is_arrived,
    is_behind,
    latlon_distance,
    passed_along,
    project_on_route,
    route_offset_limit,
    select_carrot_goal,
    select_path_goal,
    select_route_goal,
    smooth,
)
from robot_mission_planner.follower.route import Route, load_waypoints, resolve_file

GPS_REASON_INTERSECTION = "intersection"
GPS_REASON_NO_ROAD = "no_road"
GPS_REASON_STUCK = "stuck"
GPS_REASON_ROUTE = "route"  # mode gps: the whole leg is driven on the route's waypoints
GPS_REASON_FINAL = (
    "final"  # last metres to the goal: GPS all the way in, never back to ROAD
)

# sensor_msgs/NavSatStatus.status -> the suffix the follower appends to its state (F8).
# Fixposition reports 2 for an RTK fixed solution, 1 for float, 0 for a plain GNSS fix and
# -1 for none; under trees the float fix plus the OSM centreline offset is what pushes road
# goals through the route-offset filter, so the operator has to see which one it is.
FIX_NAMES = {2: "rtk", 1: "float", 0: "gps", -1: "nofix"}

# How much the waypoint source frame -> map_frame transform has to move before the waypoints
# are re-placed (F9): 0.1 m of translation, or a rotation whose matrix moves by this much in
# the Frobenius norm (~0.04 deg). Below that it is TF noise, not a new ENU origin.
TF_SHIFT_EPS = 0.1
TF_ROTATION_EPS = 1e-3


class RoadFollower(Node):
    STATE_ROAD = 0
    STATE_GPS = 1
    STATE_IDLE = 2  # mission: waiting for a QR goal
    STATE_PLANNING = 3  # mission: route requested from route_planner / start pause
    STATE_ARRIVED = 4  # mission: at the goal, commander stopped

    def __init__(self, default_mode: str = "road_gps"):
        super().__init__("road_follower")

        self.state = self.STATE_ROAD
        self._active_intersection = None

        # --- Mode ---
        # road_gps: road following with GPS waypoints at intersections; gps: the route only;
        # road: the road only, with no route at all.
        self.declare_parameter("mode", default_mode)

        # --- Backend ---
        # "commander" (crl_commander on the NUC), "nav2" (NavigateToPose + FollowWaypoints)
        # or "follow_path" (a bare pure-pursuit controller, road following only).
        self.declare_parameter("nav_backend", "commander")
        self.declare_parameter("follow_path_action", "follow_path")
        self.declare_parameter(
            "follow_path_spacing", 0.25
        )  # m between synthetic path poses

        # --- Frames ---
        self.declare_parameter(
            "map_frame", "FP_ENU0"
        )  # fixed frame all geometry is compared in
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter(
            "earth_frame", "FP_ECEF"
        )  # ECEF frame for GPX waypoints (commander)
        self.declare_parameter(
            "utm_frame", "utm"
        )  # UTM frame for GPX waypoints (nav2 + use_utm)

        # --- Topics / services ---
        self.declare_parameter("road_points_topic", "/predicted_path_ls")
        self.declare_parameter("intersections_topic", "/intersections")
        self.declare_parameter("gps_fix_topic", "/fixposition/odometry_llh")
        self.declare_parameter(
            "gps_filtered_topic", ""
        )  # optional second NavSatFix (telemetry)
        self.declare_parameter(
            "goal_waypoint_topic", "/goal_waypoint"
        )  # commander: operator goal
        self.declare_parameter(
            "goal_sequence_topic", "/goal_sequence"
        )  # commander: latched PoseArray
        self.declare_parameter("commander_state_topic", "/crl_commander/state")
        self.declare_parameter(
            "state_topic", "~/state"
        )  # latched String: ROAD | GPS:<reason>
        # latched PoseStamped in map_frame of the intersection that triggered GPS mode
        # (empty frame_id = none); the map_data viewer draws the enter/exit circles around it
        self.declare_parameter("active_intersection_topic", "~/active_intersection")
        # Trigger service that stops the current leg from any state (F7): commander STOP,
        # every pending timer cancelled, back to IDLE. Cheaper than the e-stop, which costs
        # 5 points in the competition.
        self.declare_parameter("abort_service", "~/abort")
        self.declare_parameter("switch_mode_service", "/crl_commander/switch_mode")
        self.declare_parameter(
            "configure_sequence_service", "/crl_commander/configure_sequence_mode"
        )
        self.declare_parameter("markers_topic", "gps_waypoints_markers")
        # Sphere diameter of a route waypoint, m. Waypoints are ~3 m apart, so anything
        # near that merges them into a tube that hides the robot and the road.
        self.declare_parameter("waypoint_marker_scale", 0.8)

        # --- GPS following ---
        self.declare_parameter("file", "")
        self.declare_parameter("robot_id", "helhest-robot")
        self.declare_parameter("start", 0)
        self.declare_parameter("reverse", False)
        self.declare_parameter(
            "loop", False
        )  # file routes only; mission routes never loop
        self.declare_parameter("use_utm", True)  # nav2 backend only
        self.declare_parameter("telemetry_url", "")  # empty = no telemetry POSTs

        # --- Thresholds ---
        self.declare_parameter("intersection_enter_threshold", 3.0)  # m: ROAD -> GPS
        self.declare_parameter(
            "intersection_exit_threshold", 4.0
        )  # m: from all intersections
        # Only intersections this close (m) to the planned route take part in the enter/exit
        # decisions (P4): a ring on a side junction the route merely drives past is not ours
        # (273 rings in kralovska_obora, 16 m apart). 0 = every ring counts, as before.
        self.declare_parameter("intersection_route_max_offset", 3.0)
        self.declare_parameter("gps_goal_threshold", 3.0)  # m: waypoint reached
        self.declare_parameter(
            "lookahead_sync_window", 15
        )  # waypoints searched for the closest
        self.declare_parameter(
            "road_goal_update_distance", 1.0
        )  # m: re-send active road goal
        # GPS -> ROAD hysteresis: require having passed the intersection along the route
        # direction (not just a radius), and/or a number of waypoints advanced since entry.
        self.declare_parameter("gps_exit_require_passed", True)
        self.declare_parameter("gps_exit_min_waypoints", 0)
        # Legacy: additionally require being within gps_goal_threshold of the current waypoint.
        self.declare_parameter("require_waypoint_reached_to_exit_gps", False)
        # Road-goal sanity: reject goals farther than this from the planned route (0 = off)
        # or behind the robot, so a bad segmentation cannot pull us off the mission.
        self.declare_parameter("road_goal_max_route_offset", 5.0)
        # The robot itself drives up to ~5 m off the OSM centreline under trees, so the
        # limit is relative: a goal may be margin farther off the route than the robot is,
        # never more than the hard limit (0 = no hard limit).
        self.declare_parameter("road_goal_route_offset_margin", 2.0)
        self.declare_parameter("road_goal_max_route_offset_hard", 10.0)
        self.declare_parameter("road_goal_reject_behind", True)
        # Where the ROAD goal comes from: "carrot" = one road-centre point per lidar frame
        # (convex-hull centre from build_point_cloud), "path" = the fitted/extrapolated
        # /predicted_path_ls from path_predictor, "route" = the carrot projected on the
        # planned OSM route and stretched route_stretch_distance further along it.
        self.declare_parameter("road_goal_source", "carrot")
        self.declare_parameter("carrot_topic", "/cloud_hull_center_marker")
        self.declare_parameter("carrot_type", "marker")  # marker | path (last pose)
        # The goal is never closer than min_ahead (commander arrival box is 2.5 m) and a
        # candidate farther than max_ahead is discarded as a projection artefact.
        self.declare_parameter("road_goal_min_ahead", 4.0)
        self.declare_parameter("road_goal_max_ahead", 12.0)
        self.declare_parameter(
            "road_goal_smoothing", 0.0
        )  # carrot: 0 = raw, 0.8 = damped
        # Commander backend: forget the active road goal once this close to it, so the
        # next observation re-sends one (the commander's own arrival box is 2.5 m).
        self.declare_parameter("road_goal_reached_distance", 2.5)
        # road_goal_source "route": how far along the planned route (m) past the robot /
        # carrot projection the goal is placed, how much of the carrot's lateral offset from
        # the route is carried over to it, the sharpest corner (deg) the stretch may reach
        # past (0 = no limit) and how many waypoints around the current index are searched
        # when projecting (0 = the whole route; a route folding back on itself needs a
        # window). Without a carrot the goal keeps the robot's own offset instead, which
        # drives the mapped route blind: off by default.
        self.declare_parameter("route_stretch_distance", 6.0)
        self.declare_parameter("route_lateral_gain", 1.0)
        self.declare_parameter("route_stretch_max_turn", 45.0)
        self.declare_parameter("route_projection_window", 10)
        self.declare_parameter("route_goal_without_carrot", False)
        # Failure handling
        self.declare_parameter(
            "road_path_timeout", 5.0
        )  # s without a road path -> GPS (0 = off)
        self.declare_parameter(
            "stuck_fallback_to_gps", True
        )  # commander STUCK in ROAD -> GPS
        self.declare_parameter(
            "service_timeout", 3.0
        )  # s: commander service call watchdog
        self.declare_parameter(
            "gps_sequence_window", 0
        )  # waypoints per sequence (0 = all)
        # ROAD <-> GPS hand-over (commander backend). The commander's own mode transition
        # cancels the old goal, so the new goal is sent directly (goto <-> sequence) with no
        # STOP in between; stop_between_modes restores the old STOP + delay behaviour and
        # transition_delay adds a pause before the new goal is sent (s).
        self.declare_parameter("stop_between_modes", False)
        self.declare_parameter("transition_delay", 0.0)
        # A gap this long (s) on commander_state_topic means the commander was restarted:
        # its sequence source is back at the launch default and the goal is gone, so the
        # sequence is re-configured and the current goal re-sent.
        self.declare_parameter("commander_restart_gap", 5.0)
        # The waypoint frame -> map_frame transform is looked up again every this many
        # seconds (F9): a Fixposition restart moves FP_ENU0 and every waypoint placed in it
        # would be wrong until the follower is restarted. 0 = resolve once, as before.
        self.declare_parameter("waypoint_tf_recheck_period", 10.0)

        # --- Mission (Robotour): QR goal -> route_planner -> follow -> arrive -> idle ---
        # With no `file`, the follower idles until a goal arrives on qr_goal_topic, asks
        # plan_route_action for a route from its own GNSS fix to it, pauses start_delay,
        # follows, and once within goal_reached_radius of the last waypoint stops and idles
        # again. QR goals received while not IDLE are ignored.
        self.declare_parameter(
            "qr_goal_topic", "/qr_goal/goal"
        )  # GeoPointStamped, latched
        self.declare_parameter("plan_route_action", "/route_planner/plan_route")
        self.declare_parameter(
            "plan_spacing", 3.0
        )  # m between route waypoints (0 = planner default)
        self.declare_parameter("plan_retries", 3)  # attempts before giving up on a goal
        self.declare_parameter("plan_retry_delay", 5.0)  # s between attempts
        # PlanRoute failure reasons that will not change on a retry (the goal is too far
        # from any way): give up on the goal at once instead of plan_retries attempts.
        self.declare_parameter("plan_no_retry_reasons", ["snap_too_far"])
        self.declare_parameter(
            "plan_timeout", 60.0
        )  # s for one attempt (server + planning)
        self.declare_parameter(
            "start_delay", 5.0
        )  # s between route received and first command
        self.declare_parameter(
            "goal_reached_radius", 5.0
        )  # m to the last waypoint = arrived
        self.declare_parameter(
            "arrival_index_window", 3
        )  # waypoints from the end that count
        self.declare_parameter(
            "arrived_hold", 0.0
        )  # s to stay ARRIVED before IDLE (signal later)
        # Final approach: with less route left than this (m) the follower stays in GPS to the
        # last waypoint. The planner puts that waypoint on the goal coordinate itself, which
        # can be off the footway (a loading zone on a lawn), where there is no road to follow
        # and the road goal would pull the robot back onto the path. 0 = off.
        self.declare_parameter("final_approach_distance", 15.0)
        self.declare_parameter(
            "event_topic", "~/event"
        )  # latched String mission events
        # Home capture (R3): the fix at the first goal of a run is the service area, which
        # the return leg has to come back to. It is written to mission_dir (where
        # route_planner keeps its GPX files as well) and published latched on home_topic,
        # so the return goal can be sent with `qr_goal_send --home` instead of typed in.
        self.declare_parameter("home_topic", "~/home")  # latched GeoPointStamped
        self.declare_parameter("mission_dir", "~/missions")
        # qr_goal_topic is latched: after a restart the follower would receive the previous
        # goal again and drive off unprompted. Goals stamped before the node started (minus
        # this tolerance, s) are ignored; 0 = accept everything.
        self.declare_parameter("stale_goal_tolerance", 2.0)
        # A goal that arrives while the follower is busy is buffered and taken when the leg
        # ends (qr_goal suppresses the same code for republish_after_s, so it would otherwise
        # be lost). A goal this close (m) to the one being driven is the same code seen again
        # and is dropped instead.
        self.declare_parameter("pending_goal_min_distance", 2.0)

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        try:
            self.mode = modes.get(str(gp("mode")))
        except KeyError as e:
            self.get_logger().error(f"{e}; using 'road_gps'")
            self.mode = modes.ROAD_GPS
        self.nav_backend = gp("nav_backend")
        if self.nav_backend not in BACKEND_KINDS:
            self.get_logger().error(
                f"Unknown nav_backend '{self.nav_backend}' (expected one of {BACKEND_KINDS}), "
                "using 'commander'"
            )
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
        self.intersection_route_max_offset = float(gp("intersection_route_max_offset"))
        self.gps_threshold = gp("gps_goal_threshold")
        self.lookahead_sync_window = gp("lookahead_sync_window")
        self.road_goal_update_distance = gp("road_goal_update_distance")
        self.gps_exit_require_passed = gp("gps_exit_require_passed")
        self.gps_exit_min_waypoints = gp("gps_exit_min_waypoints")
        self.require_wp_to_exit = gp("require_waypoint_reached_to_exit_gps")
        self.road_goal_max_route_offset = gp("road_goal_max_route_offset")
        self.road_goal_route_offset_margin = float(gp("road_goal_route_offset_margin"))
        self.road_goal_max_route_offset_hard = float(
            gp("road_goal_max_route_offset_hard")
        )
        self.road_goal_reject_behind = gp("road_goal_reject_behind")
        self.road_goal_source = gp("road_goal_source")
        if self.road_goal_source not in ("carrot", "path", "route"):
            self.get_logger().error(
                f"Unknown road_goal_source '{self.road_goal_source}', using 'carrot'"
            )
            self.road_goal_source = "carrot"
        self.carrot_type = gp("carrot_type")
        self.road_goal_min_ahead = gp("road_goal_min_ahead")
        self.road_goal_max_ahead = gp("road_goal_max_ahead")
        self.road_goal_smoothing = gp("road_goal_smoothing")
        self.road_goal_reached_distance = gp("road_goal_reached_distance")
        self.route_stretch_distance = float(gp("route_stretch_distance"))
        self.route_lateral_gain = float(gp("route_lateral_gain"))
        self.route_stretch_max_turn = math.radians(float(gp("route_stretch_max_turn")))
        self.route_projection_window = int(gp("route_projection_window"))
        self.route_goal_without_carrot = bool(gp("route_goal_without_carrot"))
        self.road_path_timeout = gp("road_path_timeout")
        self.stuck_fallback_to_gps = gp("stuck_fallback_to_gps")
        self.service_timeout = gp("service_timeout")
        self.gps_sequence_window = gp("gps_sequence_window")
        self.stop_between_modes = bool(gp("stop_between_modes"))
        self.transition_delay = float(gp("transition_delay"))
        self.commander_restart_gap = float(gp("commander_restart_gap"))
        self.waypoint_tf_recheck_period = float(gp("waypoint_tf_recheck_period"))
        self.plan_spacing = float(gp("plan_spacing"))
        self.plan_retries = int(gp("plan_retries"))
        self.plan_retry_delay = float(gp("plan_retry_delay"))
        self.plan_no_retry_reasons = {
            str(r) for r in (gp("plan_no_retry_reasons") or []) if r
        }
        self.plan_timeout = float(gp("plan_timeout"))
        self.start_delay = float(gp("start_delay"))
        self.goal_reached_radius = float(gp("goal_reached_radius"))
        self.arrival_index_window = int(gp("arrival_index_window"))
        self.arrived_hold = float(gp("arrived_hold"))
        self.final_approach_distance = float(gp("final_approach_distance"))
        self.stale_goal_tolerance = float(gp("stale_goal_tolerance"))
        self.pending_goal_min_distance = float(gp("pending_goal_min_distance"))
        self.mission_dir = str(gp("mission_dir"))

        # --- TF ---
        # Waypoints are sent in the backend's own frame (ECEF for the commander, UTM for
        # nav2) and placed in map_frame through TF; nav2 without use_utm sends lat/lon and
        # needs no transform at all.
        self.waypoint_src_frame = (
            self.earth_frame if self.nav_backend == "commander" else self.utm_frame
        )
        self.frames = Frames(
            self,
            map_frame=self.map_frame,
            robot_frame=self.robot_frame,
            source_frame=self.waypoint_src_frame,
        )

        # --- Backend I/O ---
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        if self.nav_backend == "commander":
            self.backend = make_backend(
                "commander",
                self,
                self.frames,
                earth_frame=self.earth_frame,
                goal_waypoint_topic=gp("goal_waypoint_topic"),
                goal_sequence_topic=gp("goal_sequence_topic"),
                switch_mode_service=gp("switch_mode_service"),
                configure_sequence_service=gp("configure_sequence_service"),
                state_topic=gp("commander_state_topic"),
                restart_gap=self.commander_restart_gap,
                service_timeout=self.service_timeout,
            )
        elif self.nav_backend == "follow_path":
            self.backend = make_backend(
                "follow_path",
                self,
                self.frames,
                action_name=gp("follow_path_action"),
                path_spacing=gp("follow_path_spacing"),
            )
        else:
            self.backend = make_backend(
                "nav2",
                self,
                self.frames,
                use_utm=self.use_utm,
                road_reached_distance=self.gps_threshold,
            )
            self.backend.on_sequence_succeeded = self._sequence_succeeded
        self.backend.on_goal_inactive = self._backend_goal_inactive
        self.backend.on_waypoint_reached = self._backend_waypoint_reached
        if self.mode.route and not self.backend.supports_sequence:
            self.get_logger().error(
                f"nav_backend '{self.nav_backend}' cannot drive a waypoint sequence, which "
                f"mode {self.mode.name} needs: use mode road, or another backend."
            )
        if not self.mode.route and self.road_goal_source == "route":
            self.get_logger().warning(
                f"road_goal_source 'route' needs a planned route, which mode {self.mode.name} "
                "does not have: falling back to the plain carrot."
            )

        self._waypoint_marker_scale = float(gp("waypoint_marker_scale"))
        self._marker_pub = self.create_publisher(MarkerArray, gp("markers_topic"), 10)
        self._state_pub = self.create_publisher(String, gp("state_topic"), latched)
        self._event_pub = self.create_publisher(String, gp("event_topic"), latched)
        self._home_pub = self.create_publisher(
            GeoPointStamped, gp("home_topic"), latched
        )
        self._active_int_pub = self.create_publisher(
            PoseStamped, gp("active_intersection_topic"), latched
        )
        self._abort_srv = self.create_service(
            Trigger, gp("abort_service"), self._abort_callback
        )
        self._published_active = (
            object()
        )  # sentinel so the first state is always published
        self.create_timer(5.0, self._publish_waypoints_markers)

        # --- Waypoints ---
        self.route = Route()
        self.route.index = self.start_index
        self.waypoints = []  # backend goal messages (PoseStamped in src frame, or GeoPose)
        self.gps_path = ""
        file_points = self._load_gps_data() if self.mode.route else []
        if file_points:
            self._set_route(file_points, f"file {self.gps_path}")
            if not self.mode.road:
                # Nothing else to drive: the route from the start (the reason is set with the
                # rest of the runtime state below).
                self.state = self.STATE_GPS
        elif self.mode.route:
            self.state = self.STATE_IDLE  # mission: wait for a goal to plan a route to

        if self._geo_goals:
            self._process_waypoints()  # lat/lon goals, no transform needed
        else:
            self._utm_timer = self.create_timer(1.0, self._resolve_waypoint_transform)

        # --- Runtime state ---
        self._latest_road_path = None  # nav_msgs/Path (road_goal_source=path)
        self._latest_carrot = None  # (x, y) in map_frame (road_goal_source=carrot)
        self._last_road_path_time = None  # node clock seconds of the last road path
        self._intersections = None  # PoseArray as received
        self._intersections_map = None  # np.ndarray (N, 2) in map_frame
        self._intersections_on_route = None  # the subset of them on the planned route
        self._goal_active = False
        self._last_road_goal = None  # (x, y) in map_frame of the active ROAD goal
        self._pending_goal_timer = None
        self._gps_start_index = 0
        self.route.synced = False  # first sync searches the whole list
        # Why GPS mode was entered; mode gps starts in it (see the waypoints section above).
        self._gps_reason = GPS_REASON_ROUTE if self.state == self.STATE_GPS else None
        self._gps_entry_index = 0  # waypoint index when GPS mode was entered
        self._gps_route_dir = (
            None  # unit route direction leaving the active intersection
        )
        self._gps_node_index = None  # route waypoint nearest to the active intersection
        self._mission_goal = None  # (lat, lon) of the QR goal being planned / followed
        self._pending_goal = None  # (lat, lon, stamp) seen while busy, taken on IDLE
        self._home = None  # (lat, lon) of the service area, captured at the first goal
        self._plan_attempt = 0
        self._plan_goal_handle = None
        self._plan_timer = None  # retry / start-delay / timeout timer
        self._plan_started = 0.0
        self._arrived_time = 0.0
        self._fix_status = None  # last NavSatStatus.status seen on gps_fix_topic
        # R1: the team-defined "continue" signal is the next QR code shown to the robot, so
        # the first goal accepted after an arrival is announced as CONTINUE as well.
        self._continue_after_arrival = False
        self._plan_client = None

        self.pose_gps = None
        self.pose_ekf = None
        self.path_send_url = False
        self.data = {"robot_id": self.robot_id}

        # --- Subscriptions ---
        if self.mode.road:
            if self.road_goal_source == "path":
                self.create_subscription(
                    Path, gp("road_points_topic"), self._path_callback, 10
                )
            else:
                carrot_msg = Path if self.carrot_type == "path" else Marker
                self.create_subscription(
                    carrot_msg, gp("carrot_topic"), self._carrot_callback, 10
                )
        if self.mode.switching:
            self.create_subscription(
                PoseArray,
                gp("intersections_topic"),
                self._intersections_callback,
                latched,
            )
        if gp("gps_fix_topic"):
            self.create_subscription(
                NavSatFix,
                gp("gps_fix_topic"),
                self._gps_callback,
                qos_profile_sensor_data,
            )
        if gp("gps_filtered_topic"):
            self.create_subscription(
                NavSatFix,
                gp("gps_filtered_topic"),
                self._ekf_callback,
                qos_profile_sensor_data,
            )
        if gp("qr_goal_topic") and self.mode.mission:
            self.create_subscription(
                GeoPointStamped, gp("qr_goal_topic"), self._qr_goal_callback, latched
            )
            try:
                from map_data_interfaces.action import PlanRoute

                self._plan_action_type = PlanRoute
                self._plan_client = ActionClient(
                    self, PlanRoute, gp("plan_route_action")
                )
            except ImportError:
                self.get_logger().error(
                    "map_data_interfaces not found: QR goals cannot be planned (build it, "
                    "see map_data/map_data_interfaces)"
                )

        if hasattr(self.backend, "wait_for_servers"):
            self.get_logger().info(
                f"Waiting for the {self.nav_backend} action servers..."
            )
            self.backend.wait_for_servers()
        else:
            if not self.backend._cli_switch.wait_for_service(timeout_sec=5.0):
                self.get_logger().warning(
                    f"Commander service {gp('switch_mode_service')} not available yet; "
                    "mode switches will be retried when needed."
                )

        self.get_logger().info(
            f"Follower initialised in mode {self.mode.name} ({self.mode.description}).\n"
            f"(backend={self.nav_backend}, map_frame={self.map_frame}, "
            f"robot_frame={self.robot_frame}, waypoint frame={self.waypoint_src_frame}).\n"
            f"Road goal: {self.road_goal_source} "
            f"({gp('road_points_topic') if self.road_goal_source == 'path' else gp('carrot_topic')}), "
            f"ahead {self.road_goal_min_ahead}-{self.road_goal_max_ahead} m"
            + (
                f", stretched {self.route_stretch_distance} m along the route"
                if self.road_goal_source == "route"
                else ""
            )
            + "; "
            f"intersections: {gp('intersections_topic')}, "
            f"GPS file: {self.gps_file_name or '-'}, QR goals: {gp('qr_goal_topic')} -> "
            f"{gp('plan_route_action')}, start delay {self.start_delay} s, "
            f"arrival radius {self.goal_reached_radius} m\n"
            f"Thresholds: enter={self.enter_threshold} m, exit={self.exit_threshold} m, "
            f"waypoint={self.gps_threshold} m, route offset={self.road_goal_max_route_offset} m, "
            f"road path timeout={self.road_path_timeout} s"
        )
        self._start_time = self._now()
        self.create_timer(1.0, self._main_logic_step)

    # ------------------------------------------------------------------ TF helpers
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _resolve_waypoint_transform(self):
        """
        Resolve waypoint source frame -> map_frame and place the waypoints in map_frame.

        Kept running at ``waypoint_tf_recheck_period`` after the first success instead of
        cancelling the timer: a Fixposition restart re-defines FP_ENU0 (its origin is the
        first fix of the run), and every waypoint, the route polyline and the cached
        intersections would stay where the old origin put them.
        """
        what = self.frames.refresh_source()
        if what is None:
            return
        self._process_waypoints()
        self._intersections_map = None  # cached in the old map_frame
        self._intersections_on_route = None
        if what == "first":
            self.get_logger().info(
                f"Got {self.waypoint_src_frame} -> {self.map_frame} transform; "
                f"{self.number_waypoints} waypoints placed in {self.map_frame}"
            )
            # Switch from the 1 s acquisition timer to the (slower) re-check.
            self._utm_timer.cancel()
            if self.waypoint_tf_recheck_period > 0:
                self._utm_timer = self.create_timer(
                    self.waypoint_tf_recheck_period, self._resolve_waypoint_transform
                )

    # The route and the frames the follower measures everything in; the attributes below are
    # the node's own vocabulary for them.
    @property
    def waypoints_raw(self):
        return self.route.raw

    @property
    def waypoints_map(self):
        return self.route.map_xy

    @property
    def number_waypoints(self) -> int:
        return len(self.route)

    @property
    def current_waypoint_index(self) -> int:
        return self.route.index

    @current_waypoint_index.setter
    def current_waypoint_index(self, value: int):
        self.route.index = int(value)

    @property
    def _geo_goals(self) -> bool:
        """True when waypoints go out as lat/lon and never need a map_frame position."""
        return self.backend.geo_goals

    @property
    def _tf_ready(self) -> bool:
        """True once route waypoints have a position (or need none, with lat/lon goals)."""
        return self.frames.ready or self._geo_goals

    def _robot_pose(self):
        """Robot (x, y, yaw) in map_frame, or None."""
        return self.frames.robot_pose()

    def _pose_to_map(self, pose: PoseStamped):
        """(x, y) of a PoseStamped in map_frame, or None."""
        return self.frames.pose_to_map(pose)

    # ------------------------------------------------------------------ waypoints
    def _load_gps_data(self):
        """Parse the route file (GPX or YAML) into ``[{lat, lon, ele}, ...]`` (empty = none)."""
        if self.gps_file_name == "":
            self.get_logger().info(
                "No route file: waiting for QR goals (mission mode)."
            )
            return []
        search = [os.path.join(os.path.dirname(__file__), "..", "data")]
        try:
            search.append(
                os.path.join(
                    get_package_share_directory("robot_mission_planner"), "data"
                )
            )
        except Exception:
            pass
        self.gps_path = resolve_file(self.gps_file_name, search)
        if not os.path.exists(self.gps_path):
            self.get_logger().error(f"Route file {self.gps_path} does not exist!")
            return []
        try:
            points_raw = load_waypoints(self.gps_path, self.reverse)
        except Exception as e:
            self.get_logger().error(f"Failed to parse the route file: {e}")
            return []
        self.get_logger().info(
            f"Loaded {len(points_raw)} waypoints from {self.gps_path}"
        )
        return points_raw

    def _set_route(self, points_raw, source: str):
        """
        Replace the mission route (``[{lat, lon, ele}, ...]``): resets the waypoint index and
        everything derived from the list (map coordinates, route polyline, GPS bookkeeping).
        """
        from_file = source.startswith("file")
        if not from_file and self.loop:
            # A planned leg ends at its goal: wrapping the index / looping the commander's
            # sequence would send the robot back to the start after ARRIVED was missed.
            self.get_logger().info("Mission route: loop disabled")
            self.loop = False
        self.route.set(points_raw, source, index=self.start_index if from_file else 0)
        self._gps_entry_index = 0
        self._gps_route_dir = None
        self._last_road_goal = None
        self._intersections_on_route = None  # filtered against the old route
        self.waypoints = []
        if self._tf_ready:
            self._process_waypoints()
        self.get_logger().info(f"Route set: {len(self.route)} waypoints from {source}")
        self._publish_waypoints_markers()

    def _process_waypoints(self):
        """Convert raw lat/lon waypoints into backend goals and map_frame coordinates."""
        self.waypoints = [self.backend.waypoint_msg(pt) for pt in self.route.raw]
        self.route.place(self.frames, self.backend.to_src)
        self._intersections_on_route = None  # the route polyline moved

    def _waypoint_distance(self, idx, rob_xy):
        """Distance (m) from the robot to waypoint ``idx`` (inf if unknown)."""
        d = self.route.distance_to(idx, rob_xy)
        if math.isfinite(d):
            return d
        if self.pose_gps:  # lat/lon fallback (nav2 without UTM)
            target = self.waypoints_raw[idx]
            d_lat = (self.pose_gps["lat"] - target["lat"]) * 111320
            d_lon = (
                (self.pose_gps["lon"] - target["lon"])
                * 111320
                * math.cos(math.radians(target["lat"]))
            )
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
            marker.scale.x = marker.scale.y = marker.scale.z = (
                self._waypoint_marker_scale
            )
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
        pts = np.array(
            [
                [p.position.x, p.position.y, p.position.z]
                for p in self._intersections.poses
            ]
        )
        frame = self._intersections.header.frame_id
        if frame and frame != self.map_frame:
            m = self._lookup_matrix(self.map_frame, frame, timeout=0.5)
            if m is None:
                return None
            pts = pts @ m[:3, :3].T + m[:3, 3]
        self._intersections_map = pts[:, :2]
        self._intersections_on_route = None  # filtered from the previous array
        return self._intersections_map

    def _intersections_on_route_in_map(self):
        """
        The intersections that take part in the ROAD <-> GPS decisions: those at most
        ``intersection_route_max_offset`` from the planned route polyline (P4). Rings on
        side junctions the route only drives past no longer put the follower into GPS mode.
        Cached until the intersection array or the route changes; without a route (or with
        the parameter at 0) every ring counts, as before.
        """
        inter = self._intersections_in_map()
        if (
            inter is None
            or self.intersection_route_max_offset <= 0
            or not self.route.seg_a.size
        ):
            return inter
        if self._intersections_on_route is None:
            route = [p for p in self.waypoints_map if p is not None]
            keep = indices_near_polyline(
                [(float(x), float(y)) for x, y in inter],
                route,
                self.intersection_route_max_offset,
            )
            self._intersections_on_route = inter[np.asarray(keep, dtype=int)]
            self.get_logger().info(
                f"{len(keep)} of {len(inter)} intersections are within "
                f"{self.intersection_route_max_offset:.1f} m of the route; the rest are ignored."
            )
        return self._intersections_on_route

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
        """
        A new road observation arrived. Only a *usable* one (a goal ahead of the robot, on
        the route) counts as "the road is there": a carrot behind the robot or a path off
        the route must not keep ROAD mode alive with a stale goal, it should let the
        ``road_path_timeout`` fallback take the robot along the GPS route instead.
        """
        candidate = self._road_goal_candidate()
        if candidate is None or not self._road_goal_valid(
            candidate[:2], quiet=self.state != self.STATE_ROAD
        ):
            return
        self._last_road_path_time = self._now()
        if self.state == self.STATE_ROAD and self._road_goal_needs_update(
            candidate[:2]
        ):
            self._send_road_goal(candidate)

    def _road_goal_candidate(self):
        """(x, y, yaw) of the next ROAD goal in map_frame, or None."""
        pose = self._robot_pose()
        if pose is None:
            return None
        rob_xy, yaw = pose[:2], pose[2]
        if self.road_goal_source == "route":
            goal = self._route_goal_candidate(rob_xy)
            if goal is not None or len(self.route.polyline) >= 2:
                return goal  # a route is there: no silent fallback to the plain carrot
        if self.road_goal_source in ("carrot", "route"):
            if self._latest_carrot is None:
                return None
            return select_carrot_goal(
                self._latest_carrot,
                rob_xy,
                yaw,
                self.road_goal_min_ahead,
                self.road_goal_max_ahead,
            )
        if self._latest_road_path is None or not self._latest_road_path.poses:
            return None
        pts = [self._pose_to_map(p) for p in self._latest_road_path.poses]
        pts = [p for p in pts if p is not None]
        return select_path_goal(
            pts, rob_xy, yaw, self.road_goal_min_ahead, self.road_goal_max_ahead
        )

    def _route_goal_candidate(self, rob_xy):
        """
        ROAD goal from the planned route's shape (``road_goal_source: route``).

        The carrot and the robot are projected on the route, the goal is put
        ``route_stretch_distance`` further along it from whichever projects farther ahead, and
        the carrot's lateral offset from the route is carried over to it. The route is used
        relatively only: its absolute position carries the OSM error and the GNSS error (the
        robot itself drove up to 5.5 m off the mapped centreline on 2026-09-08), and both
        cancel out because the offset is re-measured against the same route every frame.
        ``None`` when there is no route yet, or no usable carrot.
        """
        if len(self.route.polyline) < 2:
            return None
        carrot = self._latest_carrot
        if carrot is not None and self.road_goal_max_ahead > 0:
            if (
                math.hypot(carrot[0] - rob_xy[0], carrot[1] - rob_xy[1])
                > self.road_goal_max_ahead
            ):
                carrot = (
                    None  # beyond the sensor range it can only be a projection artefact
                )
        if carrot is None and not self.route_goal_without_carrot:
            return None
        return select_route_goal(
            carrot,
            rob_xy,
            self.route.polyline,
            self.route.cum,
            self.current_waypoint_index,
            stretch=self.route_stretch_distance,
            min_ahead=self.road_goal_min_ahead,
            max_ahead=self.road_goal_max_ahead,
            lateral_limit=self._route_lateral_limit(rob_xy),
            lateral_gain=self.route_lateral_gain,
            max_turn=self.route_stretch_max_turn,
            window=self.route_projection_window,
        )

    def _route_lateral_limit(self, rob_xy) -> float:
        """
        How far off the route the ``route`` goal may be placed: the same relative limit the
        road-goal sanity check uses, so a robot driving 5 m off the mapped centreline (GNSS
        under trees) may keep that offset instead of being pulled back onto the OSM line.
        """
        if self.road_goal_max_route_offset <= 0 or not self.route.seg_a.size:
            return 0.0  # 0 = no clamp
        return route_offset_limit(
            distance_to_polyline(rob_xy, self.route.seg_a, self.route.seg_b),
            self.road_goal_max_route_offset,
            self.road_goal_route_offset_margin,
            self.road_goal_max_route_offset_hard,
        )

    def _road_goal_needs_update(self, goal_xy) -> bool:
        if not self._goal_active or self._last_road_goal is None:
            return True
        moved = math.hypot(
            goal_xy[0] - self._last_road_goal[0], goal_xy[1] - self._last_road_goal[1]
        )
        return moved > self.road_goal_update_distance

    def _road_path_fresh(self) -> bool:
        if self.road_path_timeout <= 0:
            return True
        if self._last_road_path_time is None:
            return False
        return (self._now() - self._last_road_path_time) < self.road_path_timeout

    def _backend_goal_inactive(self):
        """The backend has no goal being driven any more."""
        self._goal_active = False

    def _backend_waypoint_reached(self, index: int):
        if index != self.current_waypoint_index:
            self.get_logger().info(f"Backend feedback: reached waypoint {index}")
            self.current_waypoint_index = index

    def _sequence_succeeded(self):
        """nav2 finished the whole waypoint sequence."""
        if self.state == self.STATE_GPS and self.loop:
            self.current_waypoint_index = 0
            self._send_gps_goal()

    def _gps_callback(self, msg):
        self.pose_gps = {"lat": msg.latitude, "lon": msg.longitude}
        self._update_fix_status(int(msg.status.status))
        if not self.path_send_url and (
            self.pose_ekf or not self.get_parameter("gps_filtered_topic").value
        ):
            self.send_data_url("path")
            self.path_send_url = True

    def _fix_name(self) -> str:
        """Name of the current fix quality ("" until the first fix message)."""
        if self._fix_status is None:
            return ""
        return FIX_NAMES.get(self._fix_status, f"fix{self._fix_status}")

    def _update_fix_status(self, status: int):
        """Remember the fix quality and announce every change (log + FIX event)."""
        if status == self._fix_status:
            return
        self._fix_status = status
        self.get_logger().info(f"GNSS fix: {self._fix_name()} (NavSatStatus {status})")
        self._event(f"FIX:{self._fix_name()}")

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
            data["position"] = {
                "gps": self.pose_gps,
                "ekf": self.pose_ekf or self.pose_gps,
            }
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
    def _state_text(self) -> str:
        if self.state == self.STATE_ROAD:
            text = "ROAD"
        elif self.state == self.STATE_GPS:
            text = f"GPS:{self._gps_reason}"
        else:
            text = {
                self.STATE_IDLE: "IDLE",
                self.STATE_PLANNING: "PLANNING",
                self.STATE_ARRIVED: "ARRIVED",
            }[self.state]
        # F8: the fix quality rides along on the state string ("ROAD [rtk]"), so the HUD and
        # every log line that names the state say what the position is worth. Readers split
        # the state off at the first space.
        fix = self._fix_name()
        return f"{text} [{fix}]" if fix else text

    def _event(self, text: str):
        self.get_logger().info(f"EVENT {text}")
        self._event_pub.publish(String(data=text))

    def _publish_state(self):
        self._state_pub.publish(String(data=self._state_text()))
        active = self._active_intersection if self.state == self.STATE_GPS else None
        if active != self._published_active:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            if active is not None:
                msg.header.frame_id = self.map_frame
                msg.pose.position.x, msg.pose.position.y = (
                    float(active[0]),
                    float(active[1]),
                )
            msg.pose.orientation.w = 1.0
            self._active_int_pub.publish(msg)
            self._published_active = active

    def _main_logic_step(self):
        self._publish_state()
        if self.state == self.STATE_ARRIVED:
            if (self._now() - self._arrived_time) >= self.arrived_hold:
                self._enter_idle("arrived, ready for the next goal")
            return
        if self.state in (self.STATE_IDLE, self.STATE_PLANNING):
            return
        pose = self._robot_pose()
        if pose is None:
            return
        rob_xy = pose[:2]
        if self._mission_goal is not None and is_arrived(
            rob_xy,
            self.waypoints_map,
            self.current_waypoint_index,
            self.goal_reached_radius,
            self.arrival_index_window,
        ):
            self._enter_arrived(rob_xy)
            return
        # Some backends report the waypoint index and the distance to the goal through action
        # feedback (nav2); with the others the follower keeps track itself.
        if self.state == self.STATE_ROAD or not self.backend.reports_progress:
            self._sync_waypoint_index_to_closest(rob_xy)
        if not self.backend.reports_progress and self.state == self.STATE_ROAD:
            self._check_road_goal_reached(rob_xy)
        if self.backend.take_restarted():
            self._goal_active = False
            if self.state == self.STATE_GPS:
                self._send_gps_goal()
            else:
                self._send_road_goal()
        elif (
            not self.mode.switching
            and self.state == self.STATE_GPS
            and not self._goal_active
        ):
            # Nothing hands over to this state and back in the route-only mode, so the tick
            # is what starts the first sequence and picks it up again after a backend stop.
            self._send_gps_goal()
        elif self.backend.left_us(self._goal_active):
            if self.state == self.STATE_GPS:
                # The sequence window (gps_sequence_window) was consumed: send the next one.
                self.get_logger().info(
                    "Commander finished the sequence window; sending the next one."
                )
                self._send_gps_goal()
            else:
                self.get_logger().info(
                    "Commander stopped during ROAD; re-sending the road goal."
                )
                self._goal_active = False
                self._send_road_goal()
        if self.road_goal_source == "route" and self.route_goal_without_carrot:
            # Nothing else drives the road input then: no carrot arrives, so tick it from here
            # to keep the goal moving along the route and ROAD mode alive (road_path_timeout).
            self._road_input()
        self._check_state_transitions(rob_xy)

    def _sync_waypoint_index_to_closest(self, rob_xy):
        """Move current_waypoint_index to the closest waypoint within a lookahead window."""
        if not self.waypoints:
            return
        num_wps = len(self.waypoints)
        best_idx, min_dist = self.current_waypoint_index, float("inf")
        if not self.route.synced:
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
            self.route.synced = True
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
        d = math.hypot(
            rob_xy[0] - self._last_road_goal[0], rob_xy[1] - self._last_road_goal[1]
        )
        if d < self.road_goal_reached_distance:
            self.get_logger().info(
                f"Road goal reached ({d:.2f} m); the next road observation re-sends.",
                throttle_duration_sec=2.0,
            )
            self._goal_active = False

    def _closest_intersection(self, rob_xy):
        """(distance, (x, y)) of the closest intersection in map_frame, or (inf, None)."""
        inter = self._intersections_on_route_in_map()
        if inter is not None and inter.size:
            d = np.hypot(inter[:, 0] - rob_xy[0], inter[:, 1] - rob_xy[1])
            i = int(np.argmin(d))
            return float(d[i]), (float(inter[i, 0]), float(inter[i, 1]))
        if self._active_intersection is not None:
            return (
                math.hypot(
                    rob_xy[0] - self._active_intersection[0],
                    rob_xy[1] - self._active_intersection[1],
                ),
                self._active_intersection,
            )
        return float("inf"), None

    def _check_state_transitions(self, rob_xy):
        if not self.mode.switching:
            return  # gps and road drive one way from beginning to end
        if self.state not in (self.STATE_ROAD, self.STATE_GPS):
            return
        closest, closest_xy = self._closest_intersection(rob_xy)

        if self.state == self.STATE_ROAD:
            remaining = self._remaining_route_length(rob_xy)
            if remaining < self.final_approach_distance:
                self._enter_gps(
                    GPS_REASON_FINAL, None, f"{remaining:.1f} m of route left"
                )
            elif closest < self.enter_threshold:
                self._enter_gps(
                    GPS_REASON_INTERSECTION,
                    closest_xy,
                    f"approaching intersection ({closest:.2f} m)",
                )
            elif self.stuck_fallback_to_gps and self.backend.stuck():
                self._enter_gps(GPS_REASON_STUCK, None, "commander reports STUCK")
            elif (
                not self._road_path_fresh()
                and (self._now() - self._start_time) > self.road_path_timeout
            ):
                self._enter_gps(
                    GPS_REASON_NO_ROAD,
                    None,
                    f"no road path for {self.road_path_timeout} s",
                )
            return

        # STATE_GPS: decide whether to go back to road following
        if self._gps_reason == GPS_REASON_FINAL:
            # Final approach: the sequence already runs to the last waypoint = the goal, so
            # nothing takes the robot out of GPS again; only is_arrived ends this state.
            return
        if self._remaining_route_length(rob_xy) < self.final_approach_distance:
            # Ran out of route while in a fallback / intersection GPS run: keep the sequence,
            # only change the reason so the exit tests below stop applying.
            self.get_logger().info(
                "Final approach: staying in GPS mode to the last waypoint."
            )
            self._gps_reason = GPS_REASON_FINAL
            self._active_intersection = None
            return
        if (
            closest < self.enter_threshold
            and self._gps_reason != GPS_REASON_INTERSECTION
        ):
            # A fallback GPS run reached an intersection: treat it as an intersection entry.
            self._gps_reason = GPS_REASON_INTERSECTION
            self._active_intersection = closest_xy
            self._gps_entry_index = self.current_waypoint_index
            self._target_intersection(closest_xy)
            return
        if (
            closest < self.enter_threshold
            and self._gps_reason == GPS_REASON_INTERSECTION
            and self._active_intersection is not None
            and closest_xy is not None
            and math.hypot(
                closest_xy[0] - self._active_intersection[0],
                closest_xy[1] - self._active_intersection[1],
            )
            > 0.5
        ):
            # The next intersection is closer than the exit ring of the active one (rings
            # 16 m apart on average, 31 pairs under 6 m in Stromovka): adopt it if it lies
            # farther along the route, instead of leaving and re-entering GPS mode.
            next_index = self.route.nearest(closest_xy) if self.waypoints_map else None
            if (
                self._gps_node_index is None
                or next_index is None
                or next_index >= self._gps_node_index
            ):
                self.get_logger().info(
                    f"Next intersection already within {closest:.1f} m: staying in GPS mode through it."
                )
                self._active_intersection = closest_xy
                self._target_intersection(closest_xy)
                return

        advanced = self.current_waypoint_index - self._gps_entry_index
        if advanced < self.gps_exit_min_waypoints:
            return

        if self._gps_reason == GPS_REASON_INTERSECTION:
            if closest <= self.exit_threshold:
                return
            if self.gps_exit_require_passed and self._active_intersection is not None:
                index_passed = (
                    self._gps_node_index is not None
                    and self.current_waypoint_index > self._gps_node_index
                )
                if not index_passed and not passed_along(
                    rob_xy, self._active_intersection, self._gps_route_dir
                ):
                    return  # still before the intersection along the route
            if self.require_wp_to_exit and self.waypoints:
                if (
                    self._waypoint_distance(self.current_waypoint_index, rob_xy)
                    >= self.gps_threshold
                ):
                    return
            why = f"passed intersection (closest {closest:.2f} m)"
        elif self._gps_reason == GPS_REASON_STUCK:
            if self.backend.stuck() or advanced < max(1, self.gps_exit_min_waypoints):
                return
            why = "commander no longer stuck"
        else:  # GPS_REASON_NO_ROAD
            if not self._road_path_fresh() or closest <= self.exit_threshold:
                return
            why = "road path available again"

        self._enter_road(why)

    def _remaining_route_length(self, rob_xy) -> float:
        """
        Route length (m) from the robot to the last waypoint, or ``inf`` when the final
        approach does not apply: it is switched off, there is no route yet, or the route
        loops (a looping file route has no end to approach).
        """
        if self.final_approach_distance <= 0 or self.loop or not self.waypoints_map:
            return float("inf")
        return self.route.remaining_length(rob_xy)

    def _enter_gps(self, reason, intersection_xy, why):
        self.get_logger().info(f"{why}. Switching to GPS mode ({reason}).")
        self.state = self.STATE_GPS
        self._gps_reason = reason
        self._active_intersection = intersection_xy
        self._gps_entry_index = self.current_waypoint_index
        self._target_intersection(intersection_xy)
        self._hand_over("GPS")
        self._publish_state()  # do not let the state topic lag the decision by a tick

    def _target_intersection(self, intersection_xy):
        """
        Remember which route waypoint the intersection sits at and the route direction
        *leaving* it. The exit test is "passed the node along the outgoing segment"; the
        incoming direction would fail at a right-angle turn (dot product ~0) and reverse
        at a sharper one, keeping the follower in GPS mode for the rest of the leg.
        """
        if intersection_xy is not None and self.waypoints_map:
            self._gps_node_index = self.route.nearest(intersection_xy)
            self._gps_route_dir = self.route.direction_at(self._gps_node_index)
        else:
            self._gps_node_index = None
            self._gps_route_dir = self.route.direction_at(self.current_waypoint_index)

    def _enter_road(self, why):
        self.get_logger().info(f"{why}. Switching back to ROAD mode.")
        self.state = self.STATE_ROAD
        self._gps_reason = None
        self._active_intersection = None
        self._gps_route_dir = None
        self._gps_node_index = None
        self._last_road_goal = None
        self._hand_over("ROAD")
        self._publish_state()

    def _hand_over(self, mode):
        """
        Give the backend the goal of the new state. Commander backend: no STOP in between
        (its transitionTo() cancels the old goal and re-initialises the sequence itself) and
        no pause unless transition_delay says so, because every stop costs seconds at each
        of the many intersections. Nav2, or stop_between_modes: cancel first, then wait 1 s.
        """
        if not self.backend.direct_hand_over or self.stop_between_modes:
            self._cancel_current_goal()
            self._schedule_goal(delay_sec=max(1.0, self.transition_delay), mode=mode)
            return
        self._goal_active = False
        if self.transition_delay > 0.0:
            self._schedule_goal(delay_sec=self.transition_delay, mode=mode)
            return
        if self._pending_goal_timer:
            self._pending_goal_timer.cancel()
            self._pending_goal_timer = None
        if mode == "GPS":
            self._send_gps_goal()
        else:
            self._send_road_goal()

    # ------------------------------------------------------------------ mission
    def _cancel_plan_timer(self):
        if self._plan_timer is not None:
            self._plan_timer.cancel()
            self._plan_timer = None

    def _one_shot(self, delay_sec, cb):
        """Replace the pending mission timer with a one-shot ``cb`` after ``delay_sec``."""
        self._cancel_plan_timer()

        def fire():
            self._cancel_plan_timer()
            cb()

        self._plan_timer = self.create_timer(max(0.01, delay_sec), fire)

    def _qr_goal_callback(self, msg):
        lat, lon = msg.position.latitude, msg.position.longitude
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if (
            self.stale_goal_tolerance > 0
            and stamp > 0
            and stamp < self._start_time - self.stale_goal_tolerance
        ):
            self.get_logger().warning(
                f"QR goal {lat:.7f}, {lon:.7f} ignored: published {self._start_time - stamp:.0f} s "
                "before this node started (latched goal of a previous run)",
                throttle_duration_sec=30.0,
            )
            return
        self._take_goal(lat, lon, stamp)

    def _take_goal(self, lat: float, lon: float, stamp: float):
        """
        Act on a goal that passed the stale check: plan it when IDLE, else buffer it (F6).
        Called again from ``_enter_idle`` for the buffered one, so the stale check stays in
        the subscription callback - a goal that was fresh when it arrived does not go stale
        while the robot drives the leg before it.
        """
        if self.state != self.STATE_IDLE:
            self._buffer_goal(lat, lon, stamp)
            return
        if self._plan_client is None:
            self.get_logger().error(
                "QR goal received but the PlanRoute action client is unavailable"
            )
            return
        if self._continue_after_arrival:
            self._continue_after_arrival = False
            self._event("CONTINUE")
        self._event(f"GOAL:{lat:.7f},{lon:.7f}")
        self._capture_home()
        self.get_logger().info(
            f"QR goal accepted: {lat:.7f}, {lon:.7f}; requesting a route"
        )
        self._mission_goal = (lat, lon)
        self._plan_attempt = 0
        self.state = self.STATE_PLANNING
        self._publish_state()
        self._request_route()

    def _buffer_goal(self, lat: float, lon: float, stamp: float):
        """Remember a goal that arrived while the follower was busy, unless it is the goal
        of the current leg (the start code read again on the way, or at the goal itself)."""
        if (
            self._mission_goal is not None
            and latlon_distance((lat, lon), self._mission_goal)
            < self.pending_goal_min_distance
        ):
            self.get_logger().info(
                f"QR goal {lat:.7f}, {lon:.7f} ignored: it is the goal of the current leg",
                throttle_duration_sec=30.0,
            )
            return
        self._pending_goal = (lat, lon, stamp)
        self.get_logger().warning(
            f"QR goal {lat:.7f}, {lon:.7f} buffered: follower is {self._state_text()}; "
            "it is taken as soon as this leg ends",
            throttle_duration_sec=5.0,
        )

    def _capture_home(self):
        """
        Record where the run started (R3): the fix at the first accepted goal is the service
        area. Written to ``mission_dir`` as ``home_<date>.txt`` and ``home.txt`` (the file
        ``qr_goal_send --home`` reads) and published latched on ``home_topic``. Once per
        process; with no fix yet it is simply tried again at the next goal.
        """
        if self._home is not None:
            return
        if self.pose_gps is None:
            self.get_logger().warning("No GNSS fix yet: home not captured for this run")
            return
        lat, lon = self.pose_gps["lat"], self.pose_gps["lon"]
        self._home = (lat, lon)
        msg = GeoPointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "wgs84"
        msg.position.latitude, msg.position.longitude = lat, lon
        self._home_pub.publish(msg)
        written = []
        try:
            directory = os.path.expanduser(self.mission_dir)
            os.makedirs(directory, exist_ok=True)
            for name in (f"home_{time.strftime('%Y%m%d-%H%M%S')}.txt", "home.txt"):
                path = os.path.join(directory, name)
                with open(path, "w") as f:
                    f.write(f"{lat:.7f},{lon:.7f}\n")
                written.append(path)
        except OSError as e:
            self.get_logger().error(f"Could not write the home file: {e}")
        self.get_logger().info(
            f"Home captured: {lat:.7f}, {lon:.7f} -> {', '.join(written) or 'not written'}"
        )
        self._event(f"HOME:{lat:.7f},{lon:.7f}")

    def _request_route(self):
        self._plan_attempt += 1
        if not self._plan_client.server_is_ready():
            self._plan_failed("route_planner action server not available")
            return
        from geographic_msgs.msg import GeoPoint

        goal = self._plan_action_type.Goal()
        if self.pose_gps is not None:
            start = GeoPoint()
            start.latitude, start.longitude = self.pose_gps["lat"], self.pose_gps["lon"]
            goal.waypoints.append(start)
            goal.start_from_robot = False
        else:
            goal.start_from_robot = True  # let route_planner take its own fix
        end = GeoPoint()
        end.latitude, end.longitude = self._mission_goal
        goal.waypoints.append(end)
        goal.spacing = self.plan_spacing
        self._plan_started = self._now()
        self.get_logger().info(
            f"PlanRoute attempt {self._plan_attempt}/{self.plan_retries}: "
            f"{'from own fix' if not goal.start_from_robot else 'from route_planner fix'} to "
            f"{end.latitude:.7f}, {end.longitude:.7f}"
        )
        self._event("PLANNING")
        future = self._plan_client.send_goal_async(goal)
        future.add_done_callback(self._plan_response_cb)
        self._one_shot(self.plan_timeout, lambda: self._plan_failed("timeout"))

    def _plan_response_cb(self, future):
        try:
            handle = future.result()
        except Exception as e:  # noqa: BLE001
            self._plan_failed(f"send failed: {e}")
            return
        if not handle.accepted:
            self._plan_failed("rejected")
            return
        self._plan_goal_handle = handle
        handle.get_result_async().add_done_callback(self._plan_result_cb)

    def _plan_result_cb(self, future):
        if self.state != self.STATE_PLANNING:
            return
        try:
            result = future.result().result
        except Exception as e:  # noqa: BLE001
            self._plan_failed(f"result failed: {e}")
            return
        if not result.success:
            self._plan_failed(
                f"{result.reason or 'failed'}: {result.message}", reason=result.reason
            )
            return
        points = [
            {
                "lat": gp.pose.position.latitude,
                "lon": gp.pose.position.longitude,
                "ele": 0.0,
            }
            for gp in result.route.poses
        ]
        if len(points) < 2:
            self._plan_failed("empty route")
            return
        self._cancel_plan_timer()
        self._set_route(points, "route_planner")
        self._event(f"ROUTE:{len(points)} waypoints, {result.length_m:.0f} m")
        self.get_logger().info(
            f"Route received ({len(points)} waypoints, {result.length_m:.0f} m); "
            f"starting in {self.start_delay:.0f} s"
        )
        self._one_shot(self.start_delay, self._start_following)

    def _plan_failed(self, why: str, reason: str = ""):
        """
        A PlanRoute attempt failed: retry after plan_retry_delay, or give up on the goal.

        ``reason`` is the action's failure reason; one listed in plan_no_retry_reasons
        (snap_too_far: the goal is farther from any way than route_planner's
        goal_max_snap_distance) will not change on a retry and ends the goal at once.
        """
        if self.state != self.STATE_PLANNING:
            return
        self._cancel_plan_timer()
        self._plan_goal_handle = None
        final = reason in self.plan_no_retry_reasons
        if final:
            self.get_logger().warning(
                f"Route planning failed ({why}): not retrying, the goal is unusable"
            )
        if self._plan_attempt < self.plan_retries and not final:
            self.get_logger().warning(
                f"Route planning failed ({why}); retrying in {self.plan_retry_delay:.0f} s"
            )
            self._one_shot(self.plan_retry_delay, self._request_route)
            return
        self._event(f"PLAN_FAILED:{why}")
        self.get_logger().error(
            f"Route planning failed ({why}) after {self._plan_attempt} attempts"
        )
        self._enter_idle("planning failed, waiting for a new goal")

    def _start_following(self):
        if self.state != self.STATE_PLANNING:
            return
        self._event("START")
        self._enter_driving("Mission start")

    def _enter_driving(self, why: str):
        """Enter the state this mode drives in: ROAD, or GPS where there is no road following."""
        if self.mode.road:
            self._enter_road(why)
        else:
            self._enter_gps(GPS_REASON_ROUTE, None, why)

    def _enter_arrived(self, rob_xy):
        d = math.hypot(
            rob_xy[0] - self.waypoints_map[-1][0], rob_xy[1] - self.waypoints_map[-1][1]
        )
        self.get_logger().info(
            f"Goal reached ({d:.1f} m from the last waypoint): stopping."
        )
        self.state = self.STATE_ARRIVED
        self._arrived_time = self._now()
        self._cancel_plan_timer()
        if self._pending_goal_timer:
            self._pending_goal_timer.cancel()
            self._pending_goal_timer = None
        self._cancel_current_goal()
        self._gps_reason = None
        self._active_intersection = None
        self._continue_after_arrival = True
        self._event("ARRIVED")
        self._publish_state()

    def _abort_callback(self, request, response):
        """
        ``~/abort``: give up the current leg from wherever we are. The commander is stopped,
        the planning / start-delay / hand-over timers are dropped and a PlanRoute goal still
        in flight is cancelled, so nothing can revive the leg after the operator asked to
        stop. The follower is then IDLE and takes the next QR goal as usual.
        """
        left = self._state_text()
        self.get_logger().warning(f"Abort requested while {left}")
        self._cancel_plan_timer()
        if self._pending_goal_timer:
            self._pending_goal_timer.cancel()
            self._pending_goal_timer = None
        self._cancel_current_goal()
        if self._plan_goal_handle is not None:
            try:
                self._plan_goal_handle.cancel_goal_async()
            except Exception as e:  # noqa: BLE001 - the abort must never fail on this
                self.get_logger().warning(f"Could not cancel the PlanRoute goal: {e}")
            self._plan_goal_handle = None
        self._pending_goal = None  # an abort must not start the buffered leg either
        self._event(f"ABORT:{left}")
        self._enter_idle("aborted by operator")
        response.success = True
        response.message = f"aborted while {left}, follower is IDLE"
        return response

    def _enter_idle(self, why: str):
        self.get_logger().info(f"{why}. Waiting for a QR goal (IDLE).")
        self.state = self.STATE_IDLE
        self._mission_goal = None
        self._plan_goal_handle = None
        self._cancel_plan_timer()
        self._event("IDLE")
        self._publish_state()
        pending, self._pending_goal = self._pending_goal, None
        if pending is not None:
            self.get_logger().info(
                f"Taking the buffered QR goal {pending[0]:.7f}, {pending[1]:.7f}"
            )
            self._take_goal(*pending)

    # ------------------------------------------------------------------ goal dispatch
    def _schedule_goal(self, delay_sec, mode):
        if self._pending_goal_timer:
            self._pending_goal_timer.cancel()
        cb = (
            self._send_gps_goal_timer_cb
            if mode == "GPS"
            else self._send_road_goal_timer_cb
        )
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
        self.backend.cancel()
        self._goal_active = False

    def _road_goal_valid(self, goal_xy, quiet: bool = False) -> bool:
        """Sanity-check a road goal against the planned route and the robot heading."""
        pose = (
            self._robot_pose()
            if (self.road_goal_max_route_offset > 0 or self.road_goal_reject_behind)
            else None
        )
        # The "route" goal sits on the route by construction, so the offset test would always
        # pass; what it is there to catch -- a road detection that is not our road -- is the
        # carrot, and that is what is measured instead.
        checked, what = goal_xy, "goal"
        if self.road_goal_source == "route" and self._latest_carrot is not None:
            checked, what = self._latest_carrot, "carrot"
        if self.road_goal_max_route_offset > 0 and self.route.seg_a.size:
            off = distance_to_polyline(checked, self.route.seg_a, self.route.seg_b)
            off_robot = (
                distance_to_polyline(pose[:2], self.route.seg_a, self.route.seg_b)
                if pose is not None
                else None
            )
            limit = route_offset_limit(
                off_robot,
                self.road_goal_max_route_offset,
                self.road_goal_route_offset_margin,
                self.road_goal_max_route_offset_hard,
            )
            if off > limit:
                if not quiet:
                    self.get_logger().warning(
                        f"Road goal rejected: {what} {off:.1f} m off the planned route "
                        f"(> {limit:.1f} m; robot itself {off_robot if off_robot is None else round(off_robot, 1)} m off)",
                        throttle_duration_sec=2.0,
                    )
                return False
        if self.road_goal_reject_behind:
            if pose is not None and is_behind(goal_xy, pose[:2], pose[2]):
                if not quiet:
                    self.get_logger().warning(
                        "Road goal rejected: behind the robot",
                        throttle_duration_sec=2.0,
                    )
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
        self.get_logger().info(
            f"Road goal ({self.road_goal_source}): ({goal_xy[0]:.2f}, {goal_xy[1]:.2f}) "
            f"in {self.map_frame}, yaw {math.degrees(candidate[2]):.0f} deg"
            + self._road_goal_log_suffix(goal_xy),
            throttle_duration_sec=2.0,
        )
        self._goal_active = True
        self._last_road_goal = goal_xy
        self.backend.send_pose(goal_xy[0], goal_xy[1], candidate[2])

    def _road_goal_log_suffix(self, goal_xy) -> str:
        """``route`` source: how far ahead of the robot the goal is and how far off the route."""
        pose = self._robot_pose()
        if (
            self.road_goal_source != "route"
            or pose is None
            or len(self.route.polyline) < 2
        ):
            return ""
        ahead = math.hypot(goal_xy[0] - pose[0], goal_xy[1] - pose[1])
        proj = project_on_route(
            self.route.polyline,
            self.route.cum,
            goal_xy,
            self.current_waypoint_index,
            self.route_projection_window,
        )
        lateral = f"{proj[1]:+.1f}" if proj is not None else "?"
        return f" ({ahead:.1f} m ahead, {lateral} m off the route)"

    def _send_gps_goal(self):
        if self.state != self.STATE_GPS or not self.waypoints:
            return
        pose = self._robot_pose()
        if pose is not None:
            self._sync_waypoint_index_to_closest(pose[:2])
        start = self.current_waypoint_index
        # The commander starts a sequence at nearest+1 (sequence_start_from_next), so the
        # first driven goal is ~2 waypoints (6 m) past the current index. A one-waypoint
        # sequence therefore has nothing to drive to: the commander answers STOP at once and
        # the follower re-sends it every tick (the robot stands 1-2 m short of the end).
        # Always send at least the last two waypoints.
        if len(self.waypoints) >= 2:
            start = min(start, len(self.waypoints) - 2)
        end = (
            start + self.gps_sequence_window
            if self.gps_sequence_window > 0
            else len(self.waypoints)
        )
        remaining = self.waypoints[start:end]
        if not remaining:
            self.get_logger().warning(
                "No remaining GPS waypoints to send (end of mission)."
            )
            self._goal_active = False
            return
        self._gps_start_index = start
        self.get_logger().info(
            f"GPS goal: sending {len(remaining)} waypoints from index {start}."
        )
        self._goal_active = True
        if self.backend.kind == "nav2":
            self.backend.send_sequence(remaining, loop=self.loop, start_index=start)
        else:
            self.backend.send_sequence(remaining, loop=self.loop)

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
            self.get_logger().info(
                f"Saved current waypoint index {self.current_waypoint_index} to {path}"
            )
        except Exception:
            pass


def main(default_mode: str = "road_gps"):
    rclpy.init()
    node = RoadFollower(default_mode)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.save_waypoint_index()
        node.destroy_node()
        rclpy.try_shutdown()


def main_road():
    """``ros2 run robot_mission_planner road_follower_simple``: the follower in mode road."""
    main("road")


def main_gps():
    """``ros2 run robot_mission_planner gps_follower``: the follower in mode gps."""
    main("gps")


if __name__ == "__main__":
    main()
