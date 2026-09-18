"""Operator view (rviz2 + mission_hud). Only subscribes, does not drive.
Usage: ros2 launch robot_mission_planner mission_rviz.launch.py

Args:
    rviz:=false     disable rviz (run only HUD)
    hud:=false      disable HUD (run only rviz)
    fp_hud:=false   disable the fixposition status panel
    config:=<path>  custom rviz config
    params:=<path>  custom mission_hud config (default: config/mission_hud.yaml)
    fp_params:=<path>  custom fixposition_hud config (default: config/fixposition_hud.yaml)
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
    default_fp_params = os.path.join(share, "config", "fixposition_hud.yaml")

    args = [
        DeclareLaunchArgument(
            "config", default_value=default_config, description="rviz config file"
        ),
        DeclareLaunchArgument(
            "params",
            default_value=default_params,
            description="mission_hud parameter file",
        ),
        DeclareLaunchArgument(
            "fp_params",
            default_value=default_fp_params,
            description="fixposition_hud parameter file",
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("hud", default_value="true"),
        DeclareLaunchArgument("fp_hud", default_value="true"),
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
    fp_hud = Node(
        package="robot_mission_planner",
        executable="fixposition_hud",
        name="fixposition_hud",
        output="screen",
        condition=IfCondition(LaunchConfiguration("fp_hud")),
        parameters=[
            LaunchConfiguration("fp_params"),
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
    return LaunchDescription(args + [hud, fp_hud, rviz])
