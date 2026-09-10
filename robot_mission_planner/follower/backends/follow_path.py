"""A bare nav2 ``FollowPath`` controller (``path_follower``): pure pursuit, no global planner.

What ``road_follower_simple`` drove before the modes were unified. There is no planner and no
waypoint sequence here, so this backend is for ``mode: road`` only: a pose goal becomes a
straight path from the robot to it, sampled every ``path_spacing`` metres, because
path_follower walks the path from the pose nearest the robot and needs the samples in between
to measure progress. Each new path preempts the previous goal, so the robot never stops.
"""

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient

from robot_mission_planner.follower.backends.base import Backend


class FollowPathBackend(Backend):
    kind = "follow_path"
    supports_sequence = False

    def __init__(
        self,
        node,
        frames,
        *,
        action_name: str = "follow_path",
        path_spacing: float = 0.25,
    ):
        super().__init__(node, frames)
        self.path_spacing = max(float(path_spacing), 0.05)
        self._client = ActionClient(node, FollowPath, action_name or "follow_path")
        self._goal_handle = None

    def wait_for_servers(self):
        while not self._client.wait_for_server(timeout_sec=2.0):
            self.log.info(f"Still waiting for '{self._client._action_name}'...")

    # ---------------------------------------------------------------- waypoints
    def to_src(self, point):
        raise NotImplementedError(
            "follow_path drives the road, not a route of waypoints"
        )

    def waypoint_msg(self, point):
        raise NotImplementedError(
            "follow_path drives the road, not a route of waypoints"
        )

    def send_sequence(self, waypoints, loop: bool = False):
        raise NotImplementedError("follow_path cannot drive a waypoint sequence")

    # ---------------------------------------------------------------- goals
    def send_pose(self, x, y, yaw):
        robot = self.frames.robot_pose()
        if robot is None:
            self.log.warning(
                "No robot pose; cannot build a path to the goal.",
                throttle_duration_sec=5.0,
            )
            return
        goal = FollowPath.Goal()
        goal.path = self._straight_path(robot[:2], (float(x), float(y)))
        future = self._client.send_goal_async(goal, feedback_callback=self._feedback)
        future.add_done_callback(self._goal_response)

    def cancel(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    def _straight_path(self, start, end) -> Path:
        path = Path()
        path.header.stamp = self.node.get_clock().now().to_msg()
        path.header.frame_id = self.frames.map_frame
        dx, dy = end[0] - start[0], end[1] - start[1]
        steps = max(1, int(math.hypot(dx, dy) / self.path_spacing))
        yaw = math.atan2(dy, dx)
        for i in range(steps + 1):
            f = i / steps
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = start[0] + f * dx
            pose.pose.position.y = start[1] + f * dy
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        return path

    # ---------------------------------------------------------------- plumbing
    def _goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.log.error("Path rejected by the controller.")
            self.on_goal_inactive()
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._result)

    def _feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.log.info(
            f"distance_to_goal {fb.distance_to_goal:.2f} m, speed {fb.speed:.2f} m/s",
            throttle_duration_sec=2.0,
        )

    def _result(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_ABORTED:
            # Every preemption by the next path lands here by design.
            self.log.info(
                "Path aborted (preempted, or the controller gave up).",
                throttle_duration_sec=5.0,
            )
        self.on_goal_inactive()
