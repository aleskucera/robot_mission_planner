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
        (os.path.join("share", package_name, "data"), glob(os.path.join("data", "*.gpx"))),
        (os.path.join("share", package_name, "data"), glob(os.path.join("data", "*.yaml"))),
   ],
 install_requires=['setuptools', 'ros2_numpy'],
 zip_safe=True,
 maintainer=['Kucera Ales','Vlk Jan'],
 maintainer_email=['kuceral4@fel.cvut.cz','vlkjan6@fel.cvut.cz'],
 description='Robot Mission Planner',
 license='BSD-3-Clause',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'gps_follower_ros2 = robot_mission_planner.gps_follower_ros2:main',
             'road_follower = robot_mission_planner.road_follower:main',
     ],
   },
)
