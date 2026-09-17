# robot_mission_planner

ROS 2 nodes that drive a robot along a mission: visual road following, a route of GPS
waypoints, or both, switching between them at intersections.

## Nodes

### `road_follower` — the follower, in one of four modes

One node does the driving. What it uses is the `mode` parameter:

| `mode` | what drives the robot | route | intersections |
|---|---|---|---|
| `road_gps` (default) | the detected road, with the route's waypoints at intersections, when the road is lost, when the commander is stuck and for the final approach | needed | yes |
| `gps` | the route's waypoints, from beginning to end | needed | — |
| `road` | the detected road only, with no goal to arrive at | none | — |
| `gps_shift` | the route's waypoints one at a time, each moved onto the road segmented around the robot | needed | — |

**`gps_shift`** drives GPS all the way; the segmentation only corrects it. Every
`road_map_topic` message (`/road_map_2`, build_map's road grid in `map_frame`) is cropped to
`shift_radius` (6 m) around the robot, the centre band (`cost` ≤ `shift_centre_cost_max`, 0.086 =
22/255, build_map's `centerline_value_max`) is kept, and the route polyline is translated —
never rotated — onto those cells (`follower/route_shift.py`, `tests/test_route_shift.py`). On a
straight road only the lateral component comes out, so the fit cannot slide the route along
the road. The shift is clamped to `shift_max`, smoothed with `shift_smoothing`, and a map
without `shift_min_points` centre cells counts as zero, so a stale shift fades out. The goal
is the first waypoint at least `road_goal_min_ahead` ahead plus the shift, sent as a `goto`
and re-sent when it moves more than `road_goal_update_distance`. Intersections play no part:
a junction's other road biases the fit by about 1 m (2 m where the route turns), which is
less than the goal jump the earlier unshifted ring caused. The fitted cells are on
`~/shift_centre_points`.

The route of the two route modes is a GPX/YAML file (`file`, the `gps_file` launch
argument) or is planned by `route_planner` from a QR goal — that is a separate choice, not a
mode. Modes share everything below them: the frames and transforms
(`follower/frames.py`), the route and what is derived from it (`follower/route.py`), the
road-goal geometry (`follower/road_goal.py`) and the navigation backend
(`follower/backends/`, `commander` | `nav2` | `follow_path`).

The mode descriptions below are for `road_gps`, the one the Robotour mission uses.

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
attempts, `plan_retry_delay` apart, back to IDLE on failure; a `start_outside_map` (robot
off the loaded map: wrong map file?), `goal_outside_map` or `snap_too_far` answer is not
retried: `plan_no_retry_reasons`, and IDLE resumes QR detection), logs the accepted goal (an
audible signal will be added here), waits `start_delay` (5 s) and then follows the route
with the ROAD/GPS logic below. Within `goal_reached_radius` (5 m) of the last waypoint it
stops the commander, reports **ARRIVED** and returns to IDLE for the next goal. A QR goal
that arrives in any other state is buffered and taken as soon as the leg ends, unless it is
within `pending_goal_min_distance` (2 m) of the goal being driven — that is the same code
read again. States are on `~/state`, mission events
(`GOAL:lat,lon`, `HOME:lat,lon`, `PLANNING`, `ROUTE:…`, `START`, `ARRIVED`, `ABORT:<state>`,
`PLAN_FAILED:…`, `IDLE`) on `~/event`. At the **first** goal of a run the follower records its
own fix as *home* (the service area): latched on `~/home` and written to `mission_dir`
(`~/missions/home_<date>.txt` and `home.txt`), which `qr_goal_send --home` sends back as the
return goal. `~/abort` (`std_srvs/Trigger`) gives up the current leg from any state
— commander STOP, pending timers and a PlanRoute goal in flight cancelled, back to IDLE —
without the 5-point e-stop penalty. A `gps_file` bypasses all of this and follows the file
from the start.

* **ROAD** state: a goal on the visually detected road is sent to the commander (`goto`),
  re-sent only when it moved more than `road_goal_update_distance` or the previous goal was
  reached (`road_goal_reached_distance`). The goal comes from `road_goal_source`:
  `carrot` (default) drives at the convex-hull centre of the road points in the current lidar
  frame (`carrot_topic`, `/cloud_hull_center_marker` from `build_point_cloud`; or a `Path`'s
  last pose with `carrot_type=path`), `path` takes the fitted `/predicted_path_ls` from
  `path_predictor`, `route` combines the carrot with the planned OSM route (below). All of them
  keep the goal between `road_goal_min_ahead` and `road_goal_max_ahead`
  in front of the robot (a closer observation is pushed out along its bearing, a short
  predicted path is extrapolated along its last segment): `crl_commander` treats a goal inside
  its 2.5 m arrival box as already reached and would stop. The selection is pure geometry in
  `road_goal.py` (`tests/test_road_goal.py`).
  Only a *usable* observation (goal ahead of the robot and within
  `road_goal_max_route_offset` of the route) counts as "road seen": a carrot behind the
  robot or a path off the route does not keep ROAD mode alive, so `road_path_timeout`
  hands over to the GPS route instead of leaving a stale goal in the commander.
* **`road_goal_source: route`** — the hull centre is the centre of what the lidar sees, so it
  lags the robot and the plain `carrot` goal degenerates into "`road_goal_min_ahead` metres
  along the current heading": it cannot anticipate a bend, and on one it lands off the path and
  is rejected as off-route. In `route` mode the robot **and** the carrot are projected onto the
  planned route, the goal is placed `route_stretch_distance` (6 m) further along the route from
  whichever of the two projects farther ahead, and the carrot's own lateral offset from the
  route is carried over to it (`route_lateral_gain`, clamped by the `road_goal_max_route_offset`
  limits, which are relative to the robot's own offset). The map thus supplies only the *shape*
  of the road: its absolute position carries the OSM error and the GNSS error — the robot itself
  drove up to 5.5 m off the mapped centreline on 2026-09-08 — and both cancel out because the
  offset is re-measured against the same route every frame. The stretch stops before a corner
  sharper than `route_stretch_max_turn` (unless that would put the goal inside the commander's
  arrival box), `route_projection_window` waypoints bound the projection so a route folding back
  on itself is not snapped to the wrong leg, and the route-offset sanity check is applied to the
  carrot instead of the goal (which is on the route by construction). Without a route (a
  `gps_file`-less road-only run, or before the waypoint TF resolves) the mode falls back to
  `carrot`; without a carrot it sends nothing, so `road_path_timeout` still hands over to GPS
  mode — unless `route_goal_without_carrot` is set, which keeps driving the mapped route at the
  robot's own offset.
* **GPS** state: entered when the robot is within `intersection_enter_threshold` of an OSM
  intersection, when no usable road observation arrived for `road_path_timeout` seconds, or when the
  commander reports `STUCK` (`stuck_fallback_to_gps`). The next `gps_sequence_window`
  GPX waypoints are sent as a sequence. Left again once the robot is farther than
  `intersection_exit_threshold` from every intersection **and** has passed the
  intersection along the route direction (`gps_exit_require_passed`), optionally after
  `gps_exit_min_waypoints` more waypoints; fallback entries end when the road path is
  back / the commander is no longer stuck. The last `final_approach_distance` (15 m) of
  route are always driven in GPS (`GPS:final`, never left again): the route's last waypoint
  is the goal coordinate itself, which may sit off the footway where there is no road to
  follow.
* **Road-goal sanity**: goals farther than `road_goal_max_route_offset` from the planned
  GPX line or behind the robot (`road_goal_reject_behind`) are rejected, so a bad
  segmentation cannot pull the robot off the mission. Commander service calls are
  watched with `service_timeout`.

The node publishes its own state as a latched `std_msgs/String` on `state_topic`
(`/road_follower/state`: `ROAD` or `GPS:<intersection|no_road|stuck|final>`, followed by the
GNSS fix quality — `ROAD [rtk]`, `[float]`, `[gps]`, `[nofix]`, also an event `FIX:<name>` on
every change) and the intersection that
triggered GPS mode as a latched `PoseStamped` on `active_intersection_topic`
(`/road_follower/active_intersection`, empty `frame_id` when none) — the `map_data` viewer
tracker shows both.

Backends (`nav_backend`):

| backend | ROAD goal | GPS waypoints |
|---------|-----------|---------------|
| `commander` (Helhest NUC, default) | `PoseStamped` on `goal_waypoint_topic`, `switch_mode("goto")` | latched `PoseArray` in `earth_frame` (ECEF) on `goal_sequence_topic`, `configure_sequence_mode(source=topic)`, `switch_mode("sequence")` |
| `nav2` | `NavigateToPose` | `FollowWaypoints` (`FollowGPSWaypoints` when `use_utm:=false`) |

The waypoint frame → `map_frame` transform is looked up again every
`waypoint_tf_recheck_period` (10 s): a Fixposition restart re-defines `FP_ENU0`, and the
waypoints, the route polyline and the cached intersections are then placed again.

Frames are parameters: `map_frame` (fixed frame all distances are measured in, `FP_ENU0`),
`robot_frame` (`base_link`), `earth_frame` (`FP_ECEF`, commander waypoints) and `utm_frame`
(nav2 + `use_utm`). Intersections and road paths may arrive in any TF-connected frame.

The `crl_commander` service types come from the real package on the robot; a dev workspace
uses the interface-only stub in `src/crl_commander`.

## Launch

```bash
# the Robotour mission: QR goal -> planned route -> road following with GPS at intersections
ros2 launch robot_mission_planner follower.launch.py mode:=road_gps

# a file route, waypoints only
ros2 launch robot_mission_planner follower.launch.py mode:=gps gps_file:=stromovka_planned.gpx

# waypoints moved onto the segmented road (needs path_centerline's build_map -> /road_map_2)
ros2 launch robot_mission_planner follower.launch.py mode:=gps_shift

# road following alone, through the pure-pursuit controller instead of the commander
ros2 launch robot_mission_planner follower.launch.py mode:=road nav_backend:=follow_path

# the goal from the predicted path, or from the carrot stretched along the planned route
ros2 launch robot_mission_planner follower.launch.py road_goal_source:=path
ros2 launch robot_mission_planner follower.launch.py road_goal_source:=route

# a whole different parameter set
ros2 launch robot_mission_planner follower.launch.py config:=/path/to/my.yaml
```

`road_and_gps_follower.launch`, `gps_follower.launch` and `road_follower_simple.launch` are
kept as thin wrappers for `mode:=road_gps`, `mode:=gps` and `mode:=road`, so the robot's tmux
sessions and the replay scripts do not have to change; `gps_shift_follower.launch` does the same
for `mode:=gps_shift`. `ros2 run robot_mission_planner gps_follower` / `road_follower_simple` /
`gps_shift_follower` start the same node in those modes.

Every parameter lives in **`config/follower.yaml`** — that is the file to edit.
`config/modes/<mode>.yaml` is loaded on top of it and names only what the mode changes; the
launch arguments above override both, and each is applied only when given, so editing the
YAML is enough. `gps_file` is absolute or relative to `data/`.

In `mode: road` with `nav_backend: follow_path`, nothing else may drive `/follow_path` at the
same time: the commander sends an empty path there whenever it has no goal, so two clients
fight each other.

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
latched `geographic_msgs/GeoPointStamped` on `/qr_goal/goal`; `road_follower` picks it up when
idle, asks `route_planner` for a route and follows it (see the mission states above). A payload must be decoded in `confirm_frames` consecutive
processed frames and is published once (again only after `republish_after_s`, 10 s).
`road_follower` pauses camera decoding through `/qr_goal/enable` while a leg is planned or
driven and resumes it on arrival, abort or a failed plan (`qr_detection_service`, `""` to leave
it on); typed goals on `~/text` are taken either way.

CPU on the Jetson: one code per frame (`detectAndDecode`), JPEGs decoded straight to
`decode_downscale`-times smaller grey, a single OpenCV thread, and frames above `process_rate`
dropped before they are deserialized. Raise `decode_downscale` for less CPU, lower it if a code
at the distance it is shown from is not read.

```bash
ros2 launch robot_mission_planner qr_goal.launch    # reads /odin1/image/compressed
ros2 topic echo /qr_goal/detections          # every decoded payload (debug)
ros2 service call /qr_goal/enable std_srvs/srv/SetBool "{data: false}"   # pause detection

# manual entry (the loading-zone QR is handed to the team in the service area):
ros2 run robot_mission_planner qr_goal_send "geo:50.1103476,14.4159857"
ros2 run robot_mission_planner qr_goal_send 50.1103476,14.4159857 --direct   # no qr_goal running

# the return leg: the fix the follower recorded at the first goal of the run
ros2 run robot_mission_planner qr_goal_send --home            # ~/missions/home.txt, --home-file to override

# give up the current leg (commander STOP, follower back to IDLE):
ros2 service call /road_follower/abort std_srvs/srv/Trigger
```

Parameters (in `config/qr_goal.yaml`): `image_topic`, `image_transport` (`compressed` | `raw`),
`process_rate` (Hz, 2), `decode_downscale` (1 | 2 | 4 | 8), `confirm_frames`, `republish_after_s`, `goal_topic`, `text_topic`,
`detections_topic`, `publish_annotated` (`~/image_annotated` with the code outlined, for rqt),
`enabled`. The
default camera is the Odin (`/odin1/image/compressed`); the Basler
(`/camera/image_color/compressed`) is a backup that is not mounted. Parser and decoder are
pure functions in `qr_goal.py`, tested in `tests/test_qr_goal.py`.

## Arrival and continue signal

Robotour requires the robot to indicate that it has arrived, and homologation tests the
signalization together with the QR-code entry and the continue of the trial. On **ARRIVED**
the follower publishes the event and `mission_signal` plays the arrival sound (by default it
says "Arrived" through `helhest_bringup`'s `speak.py` → sound_play, so the NUC
`sound.launch` speaker has to be up; `backend: aplay` plays a wav instead, `backend: log`
only logs). The follower then holds ARRIVED for `arrived_hold` (2 s) so the state is visible
on the HUD before it goes IDLE. **The team-defined continue signal is showing the next QR
code**: the follower accepts it once it is IDLE and drives the next leg, and the first goal
after an arrival is also announced as the `CONTINUE` event.

```bash
ros2 launch robot_mission_planner mission_signal.launch
```

The event → text/sound table is in `config/mission_signal.yaml` (`speech:` for the `speak`
backend, `sounds:` for `aplay`, keyed by the event name before the first `:`); an event that
is not listed is only logged and a missing wav degrades to one warning. `speech_level:` picks
the speak topic per event — `info` → `/speak/info`, `warn` → `/speak/warn`, `error` →
`/speak/err` (`speak_info_topic`, `speak_warn_topic`, `speak_error_topic`), which `speak.py`
plays at rising volume; by default `ABORT` is `warn`, `PLAN_FAILED` is `error` and the rest
`info`. `_signal_gpio()` in
`mission_signal.py` is the hook for the light/GPIO backend. The event topic is latched and
`std_msgs/String` carries no stamp, so the first message received within `ignore_latched_s`
(1 s) of the node start is dropped as the previous run's latched event — a genuinely new
event in that first second is lost with it.

## Operator view (rviz)

```bash
ros2 launch robot_mission_planner mission_rviz.launch.py
```

`rviz/robotour.rviz` + the `mission_hud` node (parameters in `config/mission_hud.yaml`): the
Odin camera and the segmented path
across the top, the mission scene below, and the numbers as overlays on the 3D view.
Nothing in it commands the robot.

* **Top left panel** — Odin RGB (`/odin1/image/compressed`).
* **Top right panel** — `/centerline/centerline_cost`, the mono8 distance transform the
  centerline network produces: 0 at the road centre, ~252 at its edge, 255 off-road, so
  the road reads as the dark band. rviz2 takes the image transport from the topic name,
  so a compressed stream is named in full and there is no transport property to set.
* **3D view** — the Helhest URDF (`helhest_description`; live it comes from the robot
  through the NUC's static TF relay, `description:=true` starts a `robot_state_publisher`
  here for bags recorded without it), the route being followed (`/road_follower/route_path`,
  from a file or planned) and its waypoints, the current `/goal_waypoint` and GPS sequence, the active intersection, the
  OSM footway cloud and intersections from `osm_cloud`, `/terrain_occupancy` as the
  traversability costmap, `/predicted_path_ls` and the hull-centre carrot. Off by default:
  the dense `/terrain_map` cloud and the `/road_cloud` / `/road_map_2` clouds (cloudini
  transport, which only the robot has).
* **Overlays** — left: follower state, route progress, route source (`file <name>` or
  `route_planner`), commander state, last mission event, planner status, QR goal (both
  greyed out while a file route is driven: they describe the last planned mission) and an
  optional `hint` line (a parameter, off by
  default); right: e-stop (the panel turns red when it is in), control source (`ROS`,
  or `RC CONTROLLER` while `/joy` button 10, the take-over switch, is held), measured
  velocity (`/odom_2d`) and the commanded `/cmd_vel`, battery, the hottest motor
  temperature, GNSS position and fix. `mission_hud` builds
  both from the mission and robot topics, every one a parameter; route progress comes
  from the route path and tf, looked up at "latest" so a bag replay works unchanged.
  Bags recorded before `road_follower` published `~/route_path` fall back to
  `/route_planner/route_path`.
* **rviz Reset clears the OSM map for good.** Reset and a Fixed Frame change clear every
  display without subscribing again, and `osm_cloud` publishes `/osm_grid` and
  `/intersection_markers` once, latched. Untick and re-tick the *Map (osm_cloud)* group
  instead: that subscribes again and the latched copies arrive within a few seconds.

Useful arguments: `rviz:=false` (HUD only, e.g. rviz runs on a laptop), `hud:=false`,
`description:=true` (publish the URDF here, for old bags), `robot_body:=true`
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
