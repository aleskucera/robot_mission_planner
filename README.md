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

* **ROAD** state: a goal on the visually detected road is sent to the commander (`goto`),
  re-sent only when it moved more than `road_goal_update_distance` or the previous goal was
  reached (`road_goal_reached_distance`). The goal comes from `road_goal_source`:
  `carrot` (default) drives at the convex-hull centre of the road points in the current lidar
  frame (`carrot_topic`, `/cloud_hull_center_marker` from `build_point_cloud`; or a `Path`'s
  last pose with `carrot_type=path`), `path` takes the fitted `/predicted_path_ls` from
  `path_predictor`. Both keep the goal between `road_goal_min_ahead` and `road_goal_max_ahead`
  in front of the robot (a closer observation is pushed out along its bearing, a short
  predicted path is extrapolated along its last segment): `crl_commander` treats a goal inside
  its 2.5 m arrival box as already reached and would stop. The selection is pure geometry in
  `road_goal.py` (`tests/test_road_goal.py`).
  Only a *usable* observation (goal ahead of the robot and within
  `road_goal_max_route_offset` of the route) counts as "road seen": a carrot behind the
  robot or a path off the route does not keep ROAD mode alive, so `road_path_timeout`
  hands over to the GPS route instead of leaving a stale goal in the commander.
* **GPS** state: entered when the robot is within `intersection_enter_threshold` of an OSM
  intersection, when no usable road observation arrived for `road_path_timeout` seconds, or when the
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

The node publishes its own state as a latched `std_msgs/String` on `state_topic`
(`/road_follower/state`: `ROAD` or `GPS:<intersection|no_road|stuck>`) and the intersection that
triggered GPS mode as a latched `PoseStamped` on `active_intersection_topic`
(`/road_follower/active_intersection`, empty `frame_id` when none) — the `map_data` viewer
tracker shows both.

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
# predicted-path goal instead of the carrot:
ros2 launch robot_mission_planner road_and_gps_follower.launch road_goal_source:=path
```

Never run `road_follower_simple` at the same time: the commander sends an empty path to
`path_follower` whenever it has no goal, so two clients of `/follow_path` fight each other.

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

## QR goal input

Robotour hands the goal over as a QR code with a geo URI payload (`geo:lat,lon`, RFC 5870).
`qr_goal` reads the robot camera, decodes QR codes with OpenCV, and publishes the position as a
latched `geographic_msgs/GeoPointStamped` on `/route_planner/goal`, which makes `route_planner`
plan from the robot's fix to it. A payload must be decoded in `confirm_frames` consecutive
processed frames and is published once (again only after `republish_after_s`).

```bash
ros2 launch robot_mission_planner qr_goal.launch image_topic:=/camera/image_color/compressed
ros2 topic echo /qr_goal/detections          # every decoded payload (debug)
ros2 service call /qr_goal/enable std_srvs/srv/SetBool "{data: false}"   # pause detection

# manual entry (the loading-zone QR is handed to the team in the service area):
ros2 run robot_mission_planner qr_goal_send "geo:50.1103476,14.4159857"
ros2 run robot_mission_planner qr_goal_send 50.1103476,14.4159857 --direct   # no qr_goal running
```

Parameters: `image_topic`, `image_transport` (`compressed` | `raw`), `process_rate` (Hz),
`confirm_frames`, `republish_after_s`, `goal_topic`, `text_topic`, `detections_topic`,
`publish_annotated` (`~/image_annotated` with the code outlined, for rqt), `enabled`. The
camera topic default comes from the 2026-09-02 record list; verify it on the robot. Parser and
decoder are pure functions in `qr_goal.py`, tested in `tests/test_qr_goal.py`.

## Tests

```bash
PYTHONPATH=. python -m pytest tests   # pure-geometry tests, no ROS needed
```
