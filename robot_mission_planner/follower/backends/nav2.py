"""Nav2: ``NavigateToPose`` for a road goal, ``FollowWaypoints`` / ``FollowGPSWaypoints`` for a route."""

import utm
from action_msgs.msg import GoalStatus
from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowGPSWaypoints, FollowWaypoints, NavigateToPose
from rclpy.action import ActionClient

from robot_mission_planner.follower.backends.base import Backend
from robot_mission_planner.follower.frames import transform_xyz

import math


class Nav2Backend(Backend):
    """
    ``use_utm``: waypoints are sent as UTM poses (``FollowWaypoints``); otherwise as lat/lon
    (``FollowGPSWaypoints``), which needs no TF at all -- ``geo_goals``.
    """

    kind = "nav2"
    reports_progress = True

    def __init__(self, node, frames, *, use_utm: bool, road_reached_distance: float = 4.0):
        super().__init__(node, frames)
        self.use_utm = bool(use_utm)
        self.geo_goals = not self.use_utm
        self.road_reached_distance = float(road_reached_distance)
        self._road_client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        action = FollowWaypoints if self.use_utm else FollowGPSWaypoints
        name = "follow_waypoints" if self.use_utm else "follow_gps_waypoints"
        self._gps_client = ActionClient(node, action, name)
        self._goal_handle = None
        self._sequence_start = 0
        self._threshold_triggered = False
        self._in_sequence = False

    def wait_for_servers(self):
        self._road_client.wait_for_server()
        self._gps_client.wait_for_server()

    # ---------------------------------------------------------------- waypoints
    def to_src(self, point):
        e, n, _, _ = utm.from_latlon(point["lat"], point["lon"])
        return e, n, point.get("ele", 0.0)

    def waypoint_msg(self, point):
        if self.geo_goals:
            msg = GeoPose()
            msg.position.latitude = point["lat"]
            msg.position.longitude = point["lon"]
            msg.position.altitude = point["ele"]
            return msg
        msg = PoseStamped()
        msg.header.frame_id = self.frames.map_frame
        x, y, z = transform_xyz(self.frames.src_to_map, *self.to_src(point))
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        msg.pose.orientation.w = 1.0
        return msg

    # ---------------------------------------------------------------- goals
    def send_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.header.frame_id = self.frames.map_frame
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._threshold_triggered = False
        self._in_sequence = False
        self._send(self._road_client, goal, self._road_feedback)

    def send_sequence(self, waypoints, loop: bool = False, start_index: int = 0):
        if self.geo_goals:
            goal = FollowGPSWaypoints.Goal()
            goal.gps_poses = waypoints
        else:
            goal = FollowWaypoints.Goal()
            goal.poses = waypoints
        self._sequence_start = start_index
        self._in_sequence = True
        self._send(self._gps_client, goal, self._gps_feedback)

    def cancel(self):
        if self._goal_handle is not None:
            self.log.info("Cancelling current goal for state transition.")
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    def hand_over(self, to_gps: bool):
        self.cancel()  # nav2 has no direct hand-over between two action servers

    # ---------------------------------------------------------------- plumbing
    def _send(self, client, goal, feedback):
        future = client.send_goal_async(goal, feedback_callback=feedback)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.log.error("Goal rejected by action server.")
            self.on_goal_inactive()
            return
        self.log.info("Goal accepted.")
        self._goal_handle.get_result_async().add_done_callback(self._result)

    def _road_feedback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        if dist == 0.0:
            return
        if dist < self.road_reached_distance and not self._threshold_triggered:
            self._threshold_triggered = True
            self.log.info(f"Road distance threshold reached ({dist:.2f} m).")
            self.on_goal_inactive()

    def _gps_feedback(self, feedback_msg):
        self.on_waypoint_reached(self._sequence_start + feedback_msg.feedback.current_waypoint)

    def _result(self, future):
        status = future.result().status
        self.log.info(f"Goal finished with status: {status}")
        if status == GoalStatus.STATUS_ABORTED:
            self.log.warning("Goal aborted.")
        self.on_goal_inactive()
        if status == GoalStatus.STATUS_SUCCEEDED and self._in_sequence:
            self.on_sequence_succeeded()

    # Replaced by the follower: a finished waypoint sequence is its business (loop or not).
    def on_sequence_succeeded(self):
        pass
