"""Operator view of a Robotour mission: rviz2 with rviz/robotour.rviz + mission_hud.

    ros2 launch robot_mission_planner mission_rviz.launch.py

Camera and segmented path across the top, the mission scene below, follower /
route / commander on the left of the 3D view and e-stop / battery / temps / GNSS
on the right. Nothing here drives the robot; it only subscribes.

    rviz:=false         just the HUD node (someone else runs rviz, e.g. over a tunnel)
    hud:=false          just rviz (a mission_hud is already running elsewhere)
    description:=false  do not start robot_state_publisher (something else already does)
    config:=<path>      another rviz config
    params:=<path>      another mission_hud config (default: config/mission_hud.yaml)

The frames, topics and panel look of the HUD are in config/mission_hud.yaml.

The 2026-09-02 field bags carry no URDF frames, so the RobotModel display only has
something to draw when this launch brings its own robot_state_publisher. It also
turns /joint_states into the wheel transforms, stamped from the joint states
themselves, which keeps a bag replay consistent with the rest of its tf.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = get_package_share_directory("robot_mission_planner")
    default_config = os.path.join(share, "rviz", "robotour.rviz")
    default_params = os.path.join(share, "config", "mission_hud.yaml")

    args = [
        DeclareLaunchArgument("config", default_value=default_config,
                              description="rviz config file"),
        DeclareLaunchArgument("params", default_value=default_params,
                              description="mission_hud parameter file"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("hud", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        # The Helhest URDF, for the RobotModel display.
        DeclareLaunchArgument("description", default_value="true"),
        DeclareLaunchArgument("description_model", default_value="helhest.urdf.xacro",
                              description="xacro in helhest_description/urdf"),
    ]

    hud = Node(
        package="robot_mission_planner",
        executable="mission_hud",
        name="mission_hud",
        output="screen",
        condition=IfCondition(LaunchConfiguration("hud")),
        parameters=[LaunchConfiguration("params"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    urdf = Command(["xacro ", PathJoinSubstitution(
        [FindPackageShare("helhest_description"), "urdf", LaunchConfiguration("description_model")])])
    description = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("description")),
        parameters=[{
            "robot_description": ParameterValue(urdf, value_type=str),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
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
    return LaunchDescription(args + [hud, description, rviz])
