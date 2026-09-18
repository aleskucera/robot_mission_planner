import os
from glob import glob

from setuptools import find_packages, setup

package_name = "robot_mission_planner"

setup(
    name=package_name,
    version="0.0.1",
    # find_packages, not [package_name]: the node is built from robot_mission_planner.follower
    # and .follower.backends, and a plain (non-symlink) colcon build would leave them out.
    packages=find_packages(exclude=["tests", "tests.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch")),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.py")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "config", "modes"),
            glob(os.path.join("config", "modes", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "data"),
            glob(os.path.join("data", "*.gpx")),
        ),
        (
            os.path.join("share", package_name, "data"),
            glob(os.path.join("data", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("rviz", "*.rviz")),
        ),
    ],
    install_requires=["setuptools", "ros2_numpy"],
    zip_safe=True,
    maintainer="Kucera Ales, Vlk Jan",
    maintainer_email="kuceral4@fel.cvut.cz, vlkjan6@fel.cvut.cz",
    description="Road, GPS and mission following for the Helhest robot",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            # One follower, three modes (mode: road_gps | gps | road). The two aliases
            # start it in a mode directly, under the names the old separate nodes had.
            "road_follower = robot_mission_planner.road_follower:main",
            "gps_follower = robot_mission_planner.road_follower:main_gps",
            "gps_shift_follower = robot_mission_planner.road_follower:main_gps_shift",
            "road_follower_simple = robot_mission_planner.road_follower:main_road",
            "qr_goal = robot_mission_planner.qr_goal:main",
            "mission_hud = robot_mission_planner.mission_hud:main",
            "fixposition_hud = robot_mission_planner.fixposition_hud:main",
            "mission_signal = robot_mission_planner.mission_signal:main",
            "qr_goal_send = robot_mission_planner.qr_goal_send:main",
        ],
    },
)
