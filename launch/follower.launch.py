"""The follower, in one of its three modes.

    ros2 launch robot_mission_planner follower.launch.py mode:=road_gps
    ros2 launch robot_mission_planner follower.launch.py mode:=gps gps_file:=stromovka.gpx
    ros2 launch robot_mission_planner follower.launch.py mode:=road nav_backend:=follow_path

mode:=road_gps  the road, handing over to the route's waypoints at OSM intersections, on
                road-detection loss, on STUCK and for the final approach (Robotour, default)
mode:=gps       the route's waypoints only, never look at the road
mode:=road      the road only: no route, no intersections, no goal to arrive at

The route of the two route modes comes from gps_file (a GPX or YAML file); with no file the
follower waits for a QR goal and has route_planner plan the route to it.

config/follower.yaml holds every parameter and is the file to edit; config/modes/<mode>.yaml
is loaded on top of it and names only what that mode changes. The arguments below override
both, and only when given, so editing the YAML is enough.

The node keeps the name road_follower: mission_hud, rviz, the map_data viewer and the bag
tools all listen to /road_follower/state, /road_follower/event and the goal topics.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# launch argument -> node parameter (an argument left empty is not passed at all, so the
# config file keeps the last word)
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
