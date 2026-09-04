#!/bin/bash
# Robotour mission bag (NUC): what the QR -> route -> follow pipeline produces.
# ./rec.sh records the robot's standard topic list, which has none of these.
# Symlinked to ~/rec_robotour.sh and started from the robotour session's record window.

. $HOME/.rosrc

ros2 bag record -o "$HOME/bags/robotour_$(date +%F-%H%M%S)" \
  /tf /tf_static \
  /fixposition/odometry_llh /fixposition/odometry_enu /fixposition/ypr \
  /crl_commander/state /crl_commander/goal /crl_commander/goal_sequence /crl_commander/plan \
  /goal_waypoint /goal_sequence \
  /road_follower/state /road_follower/event /road_follower/active_intersection \
  /qr_goal/goal /qr_goal/text /qr_goal/detections \
  /route_planner/route /route_planner/route_path \
  /intersections \
  /predicted_path_ls /map_hull_center_path /map_hull_center_marker /cloud_hull_center_marker
