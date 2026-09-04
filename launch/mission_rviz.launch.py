"""Operator view of a Robotour mission: rviz2 with rviz/robotour.rviz + mission_hud.

    ros2 launch robot_mission_planner mission_rviz.launch.py

Camera and segmented path across the top, the mission scene below, follower /
route / commander on the left of the 3D view and e-stop / battery / temps / GNSS
on the right. Nothing here drives the robot; it only subscribes.

    rviz:=false         just the HUD node (someone else runs rviz, e.g. over a tunnel)
    hud:=false          just rviz (a mission_hud is already running elsewhere)
    description:=false  do not start robot_state_publisher (something else already does)
    config:=<path>      another rviz config

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

    args = [
        DeclareLaunchArgument("config", default_value=default_config,
                              description="rviz config file"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("hud", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        # Frames the HUD measures route progress in (same defaults as road_follower).
        DeclareLaunchArgument("map_frame", default_value="FP_ENU0"),
        DeclareLaunchArgument("robot_frame", default_value="base_link"),
        # Inputs; an empty topic simply leaves that line of the panel empty.
        DeclareLaunchArgument("follower_state_topic", default_value="/road_follower/state"),
        DeclareLaunchArgument("follower_event_topic", default_value="/road_follower/event"),
        DeclareLaunchArgument("commander_state_topic", default_value="/crl_commander/state"),
        DeclareLaunchArgument("planner_status_topic", default_value="/route_planner/status"),
        DeclareLaunchArgument("route_path_topic", default_value="/route_planner/route_path"),
        DeclareLaunchArgument("qr_goal_topic", default_value="/qr_goal/goal"),
        DeclareLaunchArgument("estop_topic", default_value="/estop_active"),
        DeclareLaunchArgument("battery_topic", default_value="/battery_state"),
        DeclareLaunchArgument("temperature_topic", default_value="/temperatures"),
        DeclareLaunchArgument("gps_fix_topic", default_value="/fixposition/odometry_llh"),
        # Placeholder body box from mission_hud: only useful with description:=false and
        # nothing else publishing /robot_description (the rviz display is off to match).
        DeclareLaunchArgument("robot_body", default_value="false"),
        DeclareLaunchArgument("text_size", default_value="12.0"),
        # The Helhest URDF, for the RobotModel display.
        DeclareLaunchArgument("description", default_value="true"),
        DeclareLaunchArgument("description_model", default_value="helhest.urdf.xacro",
                              description="xacro in helhest_description/urdf"),
    ]
    skip = ("config", "rviz", "hud", "description", "description_model")
    names = [a.name for a in args if a.name not in skip]

    hud = Node(
        package="robot_mission_planner",
        executable="mission_hud",
        name="mission_hud",
        output="screen",
        condition=IfCondition(LaunchConfiguration("hud")),
        parameters=[{name: LaunchConfiguration(name) for name in names}],
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
