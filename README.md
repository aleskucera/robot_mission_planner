# robot_mission_planner

ROS 2 nodes that drive a robot along a mission made of GPS waypoints, optionally switching
between visual road following and the pre-planned GPX path at intersections.

## Nodes

### `road_follower` — road following with GPS fallback at intersections

```
map_data/osm_cloud ──/intersections (PoseArray)──┐
path_centerline    ──/predicted_path_ls (Path)───┤
                                                 ▼
                                          road_follower ──► crl_commander (goto / sequence)
                                                 ▲                   or Nav2 actions
GPX mission (map_data viewer "Paths only") ──────┘
```

* **ROAD** state: the last pose of the road path is sent as the navigation goal, re-sent only
  when it moved more than `road_goal_update_distance` or the previous goal was reached.
* **GPS** state: entered when the robot is within `intersection_enter_threshold` of an OSM
  intersection, when no road path arrived for `road_path_timeout` seconds, or when the
  commander reports `STUCK` (`stuck_fallback_to_gps`). The next `gps_sequence_window`
  GPX waypoints are sent as a sequence. Left again once the robot is farther than
  `intersection_exit_threshold` from every intersection **and** has passed the
  intersection along the route direction (`gps_exit_require_passed`), optionally after
  `gps_exit_min_waypoints` more waypoints; fallback entries end when the road path is
  back / the commander is no longer stuck.
* **Road-goal sanity**: goals farther than `road_goal_max_route_offset` from the planned
  GPX line or behind the robot (`road_goal_reject_behind`) are rejected, so a bad
  segmentation cannot pull the robot off the mission. Commander service calls are
  watched with `service_timeout`.

Backends (`nav_backend`):

| backend | ROAD goal | GPS waypoints |
|---------|-----------|---------------|
| `commander` (Helhest NUC, default) | `PoseStamped` on `goal_waypoint_topic`, `switch_mode("goto")` | latched `PoseArray` in `earth_frame` (ECEF) on `goal_sequence_topic`, `configure_sequence_mode(source=topic)`, `switch_mode("sequence")` |
| `nav2` | `NavigateToPose` | `FollowWaypoints` (`FollowGPSWaypoints` when `use_utm:=false`) |

Frames are parameters: `map_frame` (fixed frame all distances are measured in, `FP_ENU0`),
`robot_frame` (`base_link`), `earth_frame` (`FP_ECEF`, commander waypoints) and `utm_frame`
(nav2 + `use_utm`). Intersections and road paths may arrive in any TF-connected frame.

The `crl_commander` service types come from the real package on the robot; a dev workspace
uses the interface-only stub in `src/crl_commander`.

### `gps_follower_ros2` — plain GPX/YAML waypoint following through Nav2 (legacy)

Telemetry POSTs are disabled in both nodes unless `telemetry_url` is set.

## Launch

```bash
ros2 launch robot_mission_planner road_and_gps_follower.launch \
    gps_file:=stromovka_planned.gpx nav_backend:=commander map_frame:=FP_ENU0
```

All frames, topics, services and thresholds are launch arguments — see
`launch/road_and_gps_follower.launch`. `gps_file` is absolute or relative to `data/`.

## Producing the mission GPX

Plan it in the `map_data` viewer (Planner → *Paths only*) and download the GPX, or with the
library; densify to ≈3 m spacing so the follower always has a nearby waypoint
(`data/stromovka_planned.gpx` was produced this way from `map_data/data/stromovka.mapdata`).

## Testing against a bag

Replay `/tf`, `/lookahead_pose` and `/fixposition/odometry_llh` from a Helhest bag, run
`map_data osm_cloud.launch.py` (geodetic mode) and this node with a stand-in commander that
serves the two services and republishes `/lookahead_pose` as `/predicted_path_ls`. Note that
rosbag2 does not reliably replay `/tf_static` to late subscribers — broadcast the bag's static
transforms separately.
