#!/usr/bin/env python3

import os
import sys

import gpxpy
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import FollowGPSWaypoints
from geographic_msgs.msg import GeoPose

class GPXFollower(Node):
    def __init__(self, gpx_file):
        super().__init__('gpx_follower')
        self._action_client = ActionClient(self, FollowGPSWaypoints, 'follow_gps_waypoints')
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
                    geo_pose = GeoPose()
                    geo_pose.position.latitude = waypoint.latitude
                    geo_pose.position.longitude = waypoint.longitude
                    geo_pose.position.altitude = waypoint.elevation if waypoint.elevation else 0.0
                    waypoints.append(geo_pose)
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

        waypoint_msg = FollowGPSWaypoints.Goal()
        waypoint_msg.gps_poses = self.waypoints

        self.get_logger().info(f"Sending {len(self.waypoints)} waypoints to FollowGPSWaypoints")
        send_goal_future = self._action_client.send_goal_async(
            waypoint_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted by server')
        result_future = goal_handle.get_result_async()
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

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Please provide the path to the GPX file as an argument")
        rclpy.shutdown()
        return

    gpx_file_path = sys.argv[1]
    gpx_file_path = os.path.join(os.path.dirname(__file__), "../../", gpx_file_path)
    print(gpx_file_path)
    if not os.path.exists(gpx_file_path):
        print(f"GPX file {gpx_file_path} does not exist")
        rclpy.shutdown()
        return

    node = GPXFollower(gpx_file_path)
    try:
        node.send_path()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
