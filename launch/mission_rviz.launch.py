"""Operator view of a Robotour mission: rviz2 with rviz/robotour.rviz + mission_hud.

    ros2 launch robot_mission_planner mission_rviz.launch.py

Camera and segmented path across the top, the mission scene below, follower /
route / commander on the left of the 3D view and e-stop / battery / temps / GNSS
on the right. Nothing here drives the robot; it only subscribes.

    rviz:=false         just the HUD node (someone else runs rviz, e.g. over a tunnel)
    hud:=false          just rviz (a mission_hud is already running elsewhere)
    config:=<path>      another rviz config
    params:=<path>      another mission_hud config (default: config/mission_hud.yaml)

The frames, topics and panel look of the HUD are in config/mission_hud.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("robot_mission_planner")
    default_config = os.path.join(share, "rviz", "robotour.rviz")
    default_params = os.path.join(share, "config", "mission_hud.yaml")

    args = [
        DeclareLaunchArgument(
            "config", default_value=default_config, description="rviz config file"
        ),
        DeclareLaunchArgument(
            "params",
            default_value=default_params,
            description="mission_hud parameter file",
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("hud", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]

    hud = Node(
        package="robot_mission_planner",
        executable="mission_hud",
        name="mission_hud",
        output="screen",
        condition=IfCondition(LaunchConfiguration("hud")),
        parameters=[
            LaunchConfiguration("params"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=["-d", LaunchConfiguration("config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    return LaunchDescription(args + [hud, rviz])
