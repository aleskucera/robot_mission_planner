#!/usr/bin/env python3

import os
import sys
import argparse

import utm
import gpxpy
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints, FollowGPSWaypoints

class GPXFollower(Node):
    def __init__(self, gpx_file, args):
        super().__init__('gpx_follower')
        self.args = args
        if args.use_gps:
            self._action_client = ActionClient(self, FollowGPSWaypoints, 'follow_gps_waypoints')
        else:
            self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.gpx_file = gpx_file
        self.parse_gpx_file()

        # Wait for the action server to be available
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for /follow_gps_waypoints action server...')

        self.get_logger().info('GPXFollower node initialized.')

    def parse_gpx_file(self):
        waypoints = []
        try:
            with open(self.gpx_file, 'r') as file:
                gpx = gpxpy.parse(file)
                for waypoint in gpx.waypoints:
                    if self.args.use_gps:
                        pose = GeoPose()
                        pose.position.latitude = waypoint.latitude
                        pose.position.longitude = waypoint.longitude
                        pose.position.altitude = waypoint.elevation if waypoint.elevation else 0.0
                    else:
                        pose = PoseStamped()
                        pose.header.frame_id = 'utm'
                        pose.header.stamp = self.get_clock().now().to_msg()
                        utm_coords = utm.from_latlon(waypoint.latitude, waypoint.longitude)
                        pose.pose.position.x, pose.pose.position.y = utm_coords[0], utm_coords[1]
                    waypoints.append(pose)
        except Exception as e:
            self.get_logger().error(f'Error parsing GPX file: {e}')
            return []
        self.waypoints = waypoints
        if not self.waypoints:
            self.get_logger().error('No waypoints found in GPX file.')
        self.get_logger().info(f'Parsed {len(self.waypoints)} waypoints from GPX file.')

    def send_path(self):
        if not self.waypoints:
            return

        if self.args.use_gps:
            waypoint_msg = FollowGPSWaypoints.Goal()
            waypoint_msg.gps_poses = self.waypoints
        else:
            waypoint_msg = FollowWaypoints.Goal()

        self.get_logger().info(f"Sending {len(self.waypoints)} waypoints to follow")
        send_goal_future = self._action_client.send_goal_async(
            waypoint_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted by server')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Current waypoint index: {feedback.current_waypoint}")

    def result_callback(self, future):
        result = future.result().result
        if result.missed_waypoints:
            self.get_logger().warn(f"Missed waypoints: {result.missed_waypoints}")
        else:
            self.get_logger().info("Successfully navigated all waypoints")
        rclpy.shutdown()

    def cancel_goal(self):
        if self.goal_handle is not None:
            self.get_logger().info("Cancelling the current goal")
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_response_callback)
        else:
            self.get_logger().warn("No goal to cancel")

    def cancel_response_callback(self, future):
        cancel_result = future.result()
        if cancel_result.goals_canceling:
            self.get_logger().info("Goal successfully canceled")
        else:
            self.get_logger().warn("Failed to cancel goal")


def parse_args():
    parser = argparse.ArgumentParser(description='GPX Follower Node')

    parser.add_argument("-f", "--file", type=str, required=True,
                        help="Path to the GPX file containing waypoints")
    parser.add_argument("--use-gps", action='store_true', help="Use GPS waypoints instead of UTM coordinates")

    return parser.parse_args()


def main(args=None):
    if args.file is None:
        print("Please provide the path to the GPX file as an argument")
        return

    gpx_file_path = args.file
    gpx_file_path = os.path.join(os.path.dirname(__file__), "../", gpx_file_path)
    if not os.path.exists(gpx_file_path):
        print(f"GPX file {gpx_file_path} does not exist")
        return

    rclpy.init(args=args)
    node = GPXFollower(gpx_file_path, args)
    try:
        node.send_path()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        node.cancel_goal()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    args = parse_args()
    main(args)
