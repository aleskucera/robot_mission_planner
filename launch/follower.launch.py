"""Follower launch script.
Usage examples:
    ros2 launch robot_mission_planner follower.launch.py mode:=road_gps
    ros2 launch robot_mission_planner follower.launch.py mode:=gps gps_file:=stromovka.gpx
    ros2 launch robot_mission_planner follower.launch.py mode:=road nav_backend:=follow_path

Modes:
  road_gps: follow road, fallback to route at intersections/errors/final approach (default)
  gps: follow route waypoints only
  road: follow road only (no route/intersections/goal)

Route comes from `gps_file` (GPX/YAML) or QR goal via `route_planner`.
Configuration: `config/follower.yaml` + `config/modes/<mode>.yaml` + launch arguments.
Node name remains `road_follower` for compatibility with existing tools.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# map launch args to node params (empty args are ignored to prefer config defaults)
OVERRIDES = {
    "gps_file": "file",
    "nav_backend": "nav_backend",
    "road_goal_source": "road_goal_source",
    "carrot_topic": "carrot_topic",
}


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("robot_mission_planner")
    mode = LaunchConfiguration("mode").perform(context)
    config = LaunchConfiguration("config").perform(context)
    mode_config = LaunchConfiguration("mode_config").perform(context) or os.path.join(
        share, "config", "modes", f"{mode}.yaml"
    )
    params = [config, mode_config, {"mode": mode}]
    for arg, name in OVERRIDES.items():
        value = LaunchConfiguration(arg).perform(context)
        if value:
            params.append({name: value})
    return [
        Node(
            package="robot_mission_planner",
            executable="road_follower",
            name=LaunchConfiguration("node_name").perform(context),
            output="screen",
            parameters=params,
        )
    ]


def generate_launch_description():
    share = get_package_share_directory("robot_mission_planner")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode", default_value="road_gps", description="road_gps | gps | road"
            ),
            DeclareLaunchArgument(
                "config", default_value=os.path.join(share, "config", "follower.yaml")
            ),
            DeclareLaunchArgument(
                "mode_config", default_value="", description="config/modes/<mode>.yaml"
            ),
            DeclareLaunchArgument(
                "gps_file", default_value="", description="GPX/YAML route file"
            ),
            DeclareLaunchArgument(
                "nav_backend",
                default_value="",
                description="commander | nav2 | follow_path",
            ),
            DeclareLaunchArgument(
                "road_goal_source",
                default_value="",
                description="carrot | path | route",
            ),
            DeclareLaunchArgument("carrot_topic", default_value=""),
            DeclareLaunchArgument("node_name", default_value="road_follower"),
            OpaqueFunction(function=launch_setup),
        ]
    )
