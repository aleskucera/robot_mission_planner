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
        if not self._goal_active:
            self._send_new_goal()

    def _send_new_goal(self):
        if self._latest_poses is None or not self._latest_poses.poses:
            self.get_logger().warn("No road points available, cannot send goal.")
            return

        pose_stamped = self._latest_poses.poses[-1]

        # pose_stamped = PoseStamped()
        # pose_stamped.header = self._latest_poses.header
        # pose_stamped.header.stamp = self.get_clock().now().to_msg()
        # pose_stamped.pose = goal_pose

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped

        self.get_logger().info(
            f"Sending goal to ({pose_stamped.pose.position.x:.2f}, {pose_stamped.pose.position.y:.2f})"
        )
        self._goal_active = True
        self._threshold_triggered = False

        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server.")
            self._goal_active = False
            self._send_new_goal()
            return

        self.get_logger().info("Goal accepted.")
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f"Distance remaining: {dist:.2f} m", throttle_duration_sec=1.0)

        if dist < self._distance_threshold and not self._threshold_triggered:
            self._threshold_triggered = True
            self.get_logger().info(
                f"Distance threshold reached ({dist:.2f} m < {self._distance_threshold} m). "
                "Cancelling goal to request a new one."
            )
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()

    def _result_callback(self, future):
        status = future.result().status
        self.get_logger().info(f"Goal finished with status: {status}")
        self._goal_active = False
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
