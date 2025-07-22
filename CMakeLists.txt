cmake_minimum_required(VERSION 3.5)
project(robot_mission_planner)

find_package(rclpy REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package()
set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclpy
  nav2_common
  nav2_core
  nav2_util
  geometry_msgs
  nav2_msgs
  pluginlib
  tf2
  tf2_geometry_msgs
  tf2_ros
)

set(library_name robot_mission_planner)

add_executable(gpx_follower scripts/gpx_follower.py)

# install(TARGETS ${library_name}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()

