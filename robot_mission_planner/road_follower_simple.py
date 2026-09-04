#!/usr/bin/env python3
"""Plain road following: forward the road-centre path to the controller, nothing else.

This is the pre-intersection behaviour (``road_follower.py`` as of commit c84aab0,
"road follower after temesvar") brought back as its own node, so the GPS/intersection
state machine in ``road_follower.py`` stays exactly as it is. Run one or the other.

Where the goal comes from is ``goal_mode``:

``path`` (the original)
    Forward the whole ``road_points_topic`` path (``/predicted_path_ls``) as the goal.
    Needs path_predictor, which needs >= num_markers_to_fit road-centre markers and
    fits a polynomial through them -- the part that misbehaves near intersections.

``carrot``
    Take a single point -- by default the convex-hull centre of the road points in the
    current frame, ``/cloud_hull_center_marker`` -- and drive straight at it: the goal
    is a synthetic path from the robot's current position to that point. No fit, no
    marker history, no extrapolation, so nothing to lose its mind; the price is that
    the robot cuts corners and aims at the centroid of whatever road is visible, which
    at a junction is the middle of the junction.

Two output modes, selected by ``output_mode``:

``follow_path`` (default)
    Send the whole path as a ``nav2_msgs/action/FollowPath`` goal to ``path_follower``,
    which pure-pursuits along it. Each new path preempts the previous goal; the
    controller does not stop in between. No global planner is involved.

``navigate_to_pose``
    The original behaviour: take the last pose of the path as a
    ``nav2_msgs/action/NavigateToPose`` goal and re-issue it once the remaining
    distance drops below ``distance_threshold``. Needs a NavigateToPose server,
    which nothing on this robot currently provides -- kept for when one returns.
"""

import math

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker

_MODES = ("follow_path", "navigate_to_pose")
_GOAL_MODES = ("path", "carrot")
_CARROT_TYPES = ("marker", "path")


class RoadFollowerSimple(Node):
    def __init__(self):
        super().__init__("road_follower_simple")

        # build_map's own centre paths end at the robot, so pure pursuit would report
        # the goal reached at once. predict_path extrapolates them ~3 m ahead, which is
        # what the controller needs to have somewhere to drive to.
        self.road_points_topic = self.declare_parameter(
            "road_points_topic", "/predicted_path_ls").value
        self.output_mode = self.declare_parameter("output_mode", "follow_path").value
        self.action_name = self.declare_parameter("action_name", "").value
        self.min_poses = self.declare_parameter("min_poses", 2).value

        # carrot mode: a single target point instead of a fitted path.
        self.goal_mode = self.declare_parameter("goal_mode", "path").value
        self.carrot_topic = self.declare_parameter(
            "carrot_topic", "/cloud_hull_center_marker").value
        # 'marker' reads Marker.pose.position (build_point_cloud's per-frame hull
        # centre, ~15 Hz); 'path' reads the last pose of a Path (build_map's
        # accumulated /map_hull_center_path, which only updates when a new marker is
        # added -- so the robot can reach it and then wait, with nothing to drive to).
        self.carrot_type = self.declare_parameter("carrot_type", "marker").value
        self.map_frame = self.declare_parameter("map_frame", "FP_ENU0").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.carrot_spacing = self.declare_parameter("carrot_spacing", 0.25).value
        # Too close: the robot would spin on the spot chasing a point under itself.
        self.min_carrot_distance = self.declare_parameter("min_carrot_distance", 1.0).value
        # Too far: beyond the sensor's reach, so it can only be a projection artefact.
        self.max_carrot_distance = self.declare_parameter("max_carrot_distance", 12.0).value
        # Exponential smoothing of the carrot in the map frame, 0 = raw, 0.9 = heavy.
        self.carrot_smoothing = self.declare_parameter("carrot_smoothing", 0.0).value

        # follow_path: don't preempt on every incoming path (they arrive at several Hz).
        # A new goal goes out once either bound is crossed.
        self.resend_period = self.declare_parameter("resend_period", 1.0).value
        self.resend_goal_move = self.declare_parameter("resend_goal_move", 0.5).value

        # navigate_to_pose: remaining distance at which the next goal is requested.
        self.distance_threshold = self.declare_parameter("distance_threshold", 2.0).value

        if self.output_mode not in _MODES:
            raise ValueError(
                f"output_mode must be one of {_MODES}, got '{self.output_mode}'")
        if self.goal_mode not in _GOAL_MODES:
            raise ValueError(
                f"goal_mode must be one of {_GOAL_MODES}, got '{self.goal_mode}'")
        if self.carrot_type not in _CARROT_TYPES:
            raise ValueError(
                f"carrot_type must be one of {_CARROT_TYPES}, got '{self.carrot_type}'")

        self._goal_handle = None
        self._goal_active = False
        self._threshold_triggered = False
        self._pending_goal_timer = None
        self._latest_path = None
        self._last_sent_time = None
        self._last_sent_goal = None
        self._carrot = None          # smoothed carrot in map_frame

        if self.output_mode == "follow_path":
            name = self.action_name or "follow_path"
            self._action_client = ActionClient(self, FollowPath, name)
        else:
            name = self.action_name or "navigate_to_pose"
            self._action_client = ActionClient(self, NavigateToPose, name)

        if self.goal_mode == "carrot":
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
            msg_type = Marker if self.carrot_type == "marker" else Path
            self.create_subscription(msg_type, self.carrot_topic, self._carrot_callback, 10)
            source = f"'{self.carrot_topic}' ({self.carrot_type})"
        else:
            self.create_subscription(Path, self.road_points_topic, self._path_callback, 10)
            source = f"'{self.road_points_topic}'"

        self.get_logger().info(f"Waiting for the '{name}' action server...")
        while not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info(f"Still waiting for '{name}'...")

        self.get_logger().info(
            f"road_follower_simple ready [{self.goal_mode} -> {self.output_mode}]: "
            f"{source} -> '{name}'")

    # -- input -------------------------------------------------------------
    def _path_callback(self, msg):
        if len(msg.poses) < self.min_poses:
            self.get_logger().warning(
                f"Path has {len(msg.poses)} poses (< min_poses={self.min_poses}), ignoring.",
                throttle_duration_sec=5.0)
            return
        self._accept_path(msg)

    def _carrot_callback(self, msg):
        """Turn one target point into a straight path from the robot to it."""
        if isinstance(msg, Path):
            if not msg.poses:
                return
            point = msg.poses[-1].pose.position
        else:
            point = msg.pose.position

        target = self._to_map_frame(point, msg.header)
        if target is None:
            return

        if self.carrot_smoothing > 0.0 and self._carrot is not None:
            a = min(max(self.carrot_smoothing, 0.0), 0.99)
            target = (a * self._carrot[0] + (1.0 - a) * target[0],
                      a * self._carrot[1] + (1.0 - a) * target[1])
        self._carrot = target

        robot = self._robot_xy()
        if robot is None:
            return

        distance = math.hypot(target[0] - robot[0], target[1] - robot[1])
        if distance < self.min_carrot_distance:
            self.get_logger().info(
                f"Carrot only {distance:.2f} m away (< {self.min_carrot_distance} m); "
                "waiting for one further ahead.", throttle_duration_sec=5.0)
            return
        if distance > self.max_carrot_distance:
            self.get_logger().warning(
                f"Carrot {distance:.2f} m away (> {self.max_carrot_distance} m); ignoring.",
                throttle_duration_sec=5.0)
            return

        self._accept_path(self._straight_path(robot, target, msg.header.stamp))

    def _accept_path(self, msg):
        self._latest_path = msg
        if self.output_mode == "follow_path":
            if self._should_resend(msg):
                self._send_goal()
        elif not self._goal_active:
            self._send_goal()

    # -- carrot helpers ----------------------------------------------------
    def _to_map_frame(self, point, header):
        """(x, y) of `point` in map_frame, or None while TF is not ready."""
        if header.frame_id == self.map_frame:
            return (point.x, point.y)

        stamped = PointStamped()
        stamped.header = header
        stamped.point = point
        for stamp in (rclpy.time.Time.from_msg(header.stamp), rclpy.time.Time()):
            try:
                tf = self._tf_buffer.lookup_transform(
                    self.map_frame, header.frame_id, stamp,
                    timeout=Duration(seconds=0.1))
            except tf2_ros.TransformException:
                continue  # the message stamp may be older than the buffer; try latest
            out = do_transform_point(stamped, tf)
            return (out.point.x, out.point.y)

        self.get_logger().warning(
            f"No transform '{header.frame_id}' -> '{self.map_frame}' for the carrot.",
            throttle_duration_sec=5.0)
        return None

    def _robot_xy(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except tf2_ros.TransformException as ex:
            self.get_logger().warning(f"No robot pose in '{self.map_frame}': {ex}",
                                   throttle_duration_sec=5.0)
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    def _straight_path(self, start, end, stamp):
        """Path from the robot to the carrot, sampled every carrot_spacing metres.

        path_follower only ever reads positions, but it walks the path from the pose
        nearest the robot, so the samples are what let it measure progress instead of
        seeing a single far-away point.
        """
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.map_frame

        dx, dy = end[0] - start[0], end[1] - start[1]
        distance = math.hypot(dx, dy)
        steps = max(1, int(distance / max(self.carrot_spacing, 0.05)))
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

    def _should_resend(self, msg):
        """True once the goal is stale in time or the path's end has moved on."""
        if self._last_sent_time is None:
            return True

        now = self.get_clock().now()
        if (now - self._last_sent_time).nanoseconds * 1e-9 >= self.resend_period:
            return True

        end = msg.poses[-1].pose.position
        prev = self._last_sent_goal
        return prev is not None and math.hypot(end.x - prev[0], end.y - prev[1]) >= self.resend_goal_move

    # -- output ------------------------------------------------------------
    def _send_goal(self):
        if self._latest_path is None or not self._latest_path.poses:
            return

        path = self._latest_path
        end = path.poses[-1]

        if self.output_mode == "follow_path":
            goal = FollowPath.Goal()
            goal.path = path
            what = f"{len(path.poses)} poses, end ({end.pose.position.x:.2f}, {end.pose.position.y:.2f})"
        else:
            end.header.stamp = self.get_clock().now().to_msg()
            goal = NavigateToPose.Goal()
            goal.pose = end
            what = f"pose ({end.pose.position.x:.2f}, {end.pose.position.y:.2f})"

        self.get_logger().info(f"Sending goal in {path.header.frame_id}: {what}",
                               throttle_duration_sec=2.0)
        self._goal_active = True
        self._threshold_triggered = False
        self._last_sent_time = self.get_clock().now()
        self._last_sent_goal = (end.pose.position.x, end.pose.position.y)

        future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Goal rejected by the action server.")
            self._goal_active = False
            self._schedule_goal(1.0)
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        if self.output_mode == "follow_path":
            self.get_logger().info(
                f"distance_to_goal {fb.distance_to_goal:.2f} m, speed {fb.speed:.2f} m/s",
                throttle_duration_sec=2.0)
            return

        # NavigateToPose: nav2 reports 0.0 in the first few feedbacks before the
        # first plan exists, which would trip the threshold immediately.
        dist = fb.distance_remaining
        self.get_logger().info(f"Distance remaining: {dist:.2f} m", throttle_duration_sec=1.0)
        if dist == 0.0 or self._threshold_triggered:
            return

        if dist < self.distance_threshold:
            self._threshold_triggered = True
            self.get_logger().info(
                f"Within {dist:.2f} m (< {self.distance_threshold} m); "
                "cancelling to request the next goal.")
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async().add_done_callback(self._cancel_done_callback)

    def _cancel_done_callback(self, _future):
        self._goal_active = False
        # A short delay lets the server tear the old goal down before the next one.
        self._schedule_goal(0.5)

    def _result_callback(self, future):
        status = future.result().status
        self._goal_active = False

        if status == GoalStatus.STATUS_SUCCEEDED:
            # In carrot mode the next carrot arrives on its own within ~0.1 s and is
            # rebuilt from the robot's new position; resending this one would only
            # succeed again immediately, spinning the action at full rate.
            if self.goal_mode == "carrot":
                self.get_logger().info("Carrot reached; waiting for the next one.",
                                       throttle_duration_sec=5.0)
                return
            self.get_logger().info("Goal succeeded; sending the next one.")
            self._send_goal()
        elif status == GoalStatus.STATUS_ABORTED:
            # In follow_path mode every preemption lands here by design.
            self.get_logger().info("Goal aborted (preempted, or the controller gave up).",
                                   throttle_duration_sec=5.0)
            if self.output_mode == "navigate_to_pose":
                self._schedule_goal(1.0)
        # STATUS_CANCELED is handled by _cancel_done_callback.

    def _schedule_goal(self, delay_sec):
        if self._pending_goal_timer is not None:
            self._pending_goal_timer.cancel()
        self._pending_goal_timer = self.create_timer(delay_sec, self._pending_goal_cb)

    def _pending_goal_cb(self):
        self._pending_goal_timer.cancel()
        self._pending_goal_timer = None
        self._send_goal()


def main():
    rclpy.init()
    node = RoadFollowerSimple()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
