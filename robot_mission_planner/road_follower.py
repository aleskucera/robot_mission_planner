#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose


class RoadFollower(Node):
    def __init__(self):
        super().__init__("road_follower")

        self.declare_parameter("distance_threshold", 2.0)
        self.declare_parameter("road_points_topic", "/road_points")

        self._distance_threshold = (
            self.get_parameter("distance_threshold").get_parameter_value().double_value
        )
        road_points_topic = (
            self.get_parameter("road_points_topic").get_parameter_value().string_value
        )

        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self._latest_poses = None
        self._goal_handle = None
        self._goal_active = False
        self._threshold_triggered = False

        self._sub = self.create_subscription(
            Path, road_points_topic, self._path_callback, 10
        )

        self.get_logger().info("Waiting for navigate_to_pose action server...")
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Still waiting for navigate_to_pose action server...")

        self.get_logger().info(
            f"Road follower initialized. Topic: {road_points_topic}, "
            f"distance threshold: {self._distance_threshold} m"
        )

    def _path_callback(self, msg):
        if not msg.poses:
            self.get_logger().warn("Received empty Path, ignoring.")
            return
        self._latest_poses = msg
        self.get_logger().warn(f"Poses {self._latest_poses.poses}")
        # if not self._goal_active:
        self._send_new_goal()

    def _send_new_goal(self):
        if self._latest_poses is None or not self._latest_poses.poses:
            self.get_logger().warn("No road points available, cannot send goal.")
            return

        pose_stamped = self._latest_poses.poses[-1]
        pose_stamped.header.stamp = self.get_clock().now().to_msg()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        self.get_logger().info(
            f"Sending goal to ({pose_stamped.pose.position.x:.2f}, {pose_stamped.pose.position.y:.2f})"
        )
        self._goal_active = True
        self._threshold_triggered = False

        self.send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        self.send_goal_future.add_done_callback(self._goal_response_callback)


    def _goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server.")
            self._goal_active = False
            # Don't immediately retry — give Nav2 time to recover
            self._schedule_new_goal(delay_sec=1.0)
            return
        self.get_logger().info("Goal accepted.")
        self.result_future = self._goal_handle.get_result_async()
        self.result_future.add_done_callback(self._result_callback)


    def _feedback_callback(self, feedback_msg):
        # BUG: For some reason you sometimes get 0.0 distance remaining in the first few feedbacks, which completely bogus.
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f"Distance remaining: {dist:.2f} m", throttle_duration_sec=1.0)
        # self.get_logger().info(f"FEEDBACK {feedback_msg.feedback}")

        if dist == 0.0:
            # ignore
            return

        if dist < self._distance_threshold and not self._threshold_triggered:
            self._threshold_triggered = True
            self.get_logger().info(
                f"Distance threshold reached ({dist:.2f} m < {self._distance_threshold} m). "
                "Cancelling goal to request a new one."
            )
            if self._goal_handle is not None:
                cancel_future = self._goal_handle.cancel_goal_async()
                # *** Hook into cancel completion instead of relying on _result_callback ***
                cancel_future.add_done_callback(self._cancel_done_callback)

    def _cancel_done_callback(self, future):
        """Called when cancel is confirmed — safe to request new goal now."""
        self.get_logger().info("Goal successfully cancelled. Scheduling new goal.")
        self._goal_active = False
        # Small delay lets Nav2 finish BT teardown before we hammer it with a new goal
        self._schedule_new_goal(delay_sec=0.5)

    def _result_callback(self, future):
        from action_msgs.msg import GoalStatus
        status = future.result().status
        self.get_logger().info(f"Goal finished with status: {status}")
        self.get_logger().info(f"Result: {future.result().result}")

        self._goal_active = False

        # Only send a new goal on genuine SUCCESS or ABORTED — not on CANCELED
        # (cancellation is handled by _cancel_done_callback)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success, sending new goal.")
            self._send_new_goal()
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("Goal aborted, scheduling retry.")
            self._schedule_new_goal(delay_sec=1.0)
        else:
            self.get_logger().error("THIS RESULT STATUS SHOULD NOT HAPPEN.")
        # STATUS_CANCELED: do nothing here — _cancel_done_callback handles it


    def _schedule_new_goal(self, delay_sec=0.5):
        if hasattr(self, '_pending_goal_timer') and self._pending_goal_timer:
            self._pending_goal_timer.cancel()
        self._pending_goal_timer = self.create_timer(delay_sec, self._send_new_goal_once)


    def _send_new_goal_once(self):
        """Timer callback wrapper — fires once then destroys itself."""
        # ROS2 timers repeat by default; cancel after first fire
        # Store timer ref to cancel it
        if hasattr(self, '_pending_goal_timer') and self._pending_goal_timer:
            self._pending_goal_timer.cancel()
            self._pending_goal_timer = None
        self._send_new_goal()


def main():
    rclpy.init()
    node = RoadFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
