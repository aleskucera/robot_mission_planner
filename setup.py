import os
from glob import glob
from setuptools import setup

package_name = 'robot_mission_planner'

setup(
 name=package_name,
 version='0.0.0',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
     (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.launch"))),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.py"))),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.yaml"))),
        (os.path.join("share", package_name, "config", "modes"),
         glob(os.path.join("config", "modes", "*.yaml"))),
        (os.path.join("share", package_name, "data"), glob(os.path.join("data", "*.gpx"))),
        (os.path.join("share", package_name, "data"), glob(os.path.join("data", "*.yaml"))),
        (os.path.join("share", package_name, "rviz"), glob(os.path.join("rviz", "*.rviz"))),
   ],
 install_requires=['setuptools', 'ros2_numpy'],
 zip_safe=True,
 maintainer=['Kucera Ales','Vlk Jan'],
 maintainer_email=['kuceral4@fel.cvut.cz','vlkjan6@fel.cvut.cz'],
 description='Robot Mission Planner',
 license='BSD-3-Clause',
 entry_points={
     'console_scripts': [
             # One follower, three modes (mode: road_gps | gps | road). The two aliases
             # start it in a mode directly, under the names the old separate nodes had.
             'road_follower = robot_mission_planner.road_follower:main',
             'gps_follower = robot_mission_planner.road_follower:main_gps',
             'road_follower_simple = robot_mission_planner.road_follower:main_road',
             'qr_goal = robot_mission_planner.qr_goal:main',
             'mission_hud = robot_mission_planner.mission_hud:main',
             'mission_signal = robot_mission_planner.mission_signal:main',
             'qr_goal_send = robot_mission_planner.qr_goal_send:main',
     ],
   },
)
