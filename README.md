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

Mission mode (no `gps_file`): the follower starts **IDLE** and waits for a QR goal
(`/qr_goal/goal`, see *QR goal input*). It then goes **PLANNING**: it asks `route_planner`'s
`PlanRoute` action for a paths-only route from its own GNSS fix to the goal (`plan_retries`
attempts, `plan_retry_delay` apart, back to IDLE on failure), logs the accepted goal (an
audible signal will be added here), waits `start_delay` (5 s) and then follows the route
with the ROAD/GPS logic below. Within `goal_reached_radius` (5 m) of the last waypoint it
stops the commander, reports **ARRIVED** and returns to IDLE for the next goal; QR goals that
arrive in any other state than IDLE are ignored. States are on `~/state`, mission events
(`GOAL:lat,lon`, `PLANNING`, `ROUTE:…`, `START`, `ARRIVED`, `PLAN_FAILED:…`, `IDLE`) on
`~/event`. A `gps_file` bypasses all of this and follows the file from the start.

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
transforms separately. `demo/run_bag_test.sh` in the workspace does all of this; `RVIZ=1`
adds the operator view below and the camera / segmentation / status topics it needs.

## QR goal input

Robotour hands the goal over as a QR code with a geo URI payload (`geo:lat,lon`, RFC 5870).
`qr_goal` reads the robot camera, decodes QR codes with OpenCV, and publishes the position as a
latched `geographic_msgs/GeoPointStamped` on `/qr_goal/goal`; `road_follower` picks it up when
idle, asks `route_planner` for a route and follows it (see the mission states above). A payload must be decoded in `confirm_frames` consecutive
processed frames and is published once (again only after `republish_after_s`).

```bash
ros2 launch robot_mission_planner qr_goal.launch    # reads /odin1/image/compressed
ros2 topic echo /qr_goal/detections          # every decoded payload (debug)
ros2 service call /qr_goal/enable std_srvs/srv/SetBool "{data: false}"   # pause detection

# manual entry (the loading-zone QR is handed to the team in the service area):
ros2 run robot_mission_planner qr_goal_send "geo:50.1103476,14.4159857"
ros2 run robot_mission_planner qr_goal_send 50.1103476,14.4159857 --direct   # no qr_goal running
```

Parameters: `image_topic`, `image_transport` (`compressed` | `raw`), `process_rate` (Hz),
`confirm_frames`, `republish_after_s`, `goal_topic`, `text_topic`, `detections_topic`,
`publish_annotated` (`~/image_annotated` with the code outlined, for rqt), `enabled`. The
default camera is the Odin (`/odin1/image/compressed`); the Basler
(`/camera/image_color/compressed`) is a backup that is not mounted. Parser and decoder are
pure functions in `qr_goal.py`, tested in `tests/test_qr_goal.py`.

## Operator view (rviz)

```bash
ros2 launch robot_mission_planner mission_rviz.launch.py
```

`rviz/robotour.rviz` + the `mission_hud` node: the Odin camera and the segmented path
across the top, the mission scene below, and the numbers as overlays on the 3D view.
Nothing in it commands the robot.

* **Top left panel** — Odin RGB (`/odin1/image/compressed`).
* **Top right panel** — `/centerline/centerline_cost`, the mono8 distance transform the
  centerline network produces: 0 at the road centre, ~252 at its edge, 255 off-road, so
  the road reads as the dark band. rviz2 takes the image transport from the topic name,
  so a compressed stream is named in full and there is no transport property to set.
* **3D view** — the Helhest URDF (`helhest_description`, started here as
  `robot_state_publisher` unless `description:=false`), the planned route and its
  waypoints, the current `/goal_waypoint` and GPS sequence, the active intersection, the
  OSM footway cloud and intersections from `osm_cloud`, `/terrain_occupancy` as the
  traversability costmap, `/predicted_path_ls` and the hull-centre carrot. Off by default:
  the dense `/terrain_map` cloud and the `/road_cloud` / `/road_map_2` clouds (cloudini
  transport, which only the robot has).
* **Overlays** — left: follower state, route progress, commander state, last mission
  event, planner status, QR goal; right: e-stop (the panel turns red when it is in),
  battery, the hottest motor temperature, GNSS position and fix. `mission_hud` builds
  both from the mission and robot topics, every one a parameter; route progress comes
  from the route path and tf, looked up at "latest" so a bag replay works unchanged.

Useful arguments: `rviz:=false` (HUD only, e.g. rviz runs on a laptop), `hud:=false`,
`description:=false` (something else publishes `/robot_description`), `robot_body:=true`
(a placeholder box instead of the URDF), `text_size:=`, `config:=`.

The dock arrangement is the `QMainWindow State` hex at the end of the config, which
`rviz/make_layout.py` regenerates — needed after renaming either Image display, since
rviz matches the dock to the display name:

```bash
python3 rviz/make_layout.py --write robotour.rviz
```

## Tests

```bash
PYTHONPATH=. python -m pytest tests   # pure-geometry tests, no ROS needed
```
