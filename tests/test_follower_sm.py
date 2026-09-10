"""
Scripted state-machine tests for ``road_follower`` against the faithful fake commander.

The 2026-09-08 field test lost minutes to two commander-side stalls that the old replay
harness could not show (review item F10): the follower's own tests covered geometry and QR
parsing only, and the fake commander finished sequence windows on a timer. These tests run
the real ``road_follower`` node against ``demo/fake_commander.py``, which emulates
crl_commander's SEQUENCE state machine (arrival box, nearest+1 start, consume-without-
advance, restart back to the launch sequence source), and a simulated robot that only moves
while the commander has a goal. No bag and no route_planner are needed: the route comes from
the follower's ``file`` parameter (a GPX written into tmp), the road carrots and the
intersections are published by the test itself.

Scenarios
  a  straight route with one ring   ROAD -> GPS:intersection -> ROAD, no STOP in between,
                                    one /goal_sequence per GPS entry (F1, F2)
  b  right-angle junction           GPS is left within 10 m after the node (F3)
  c  commander restart mid-GPS      the follower re-configures source=topic and re-sends,
                                    and the commander drives again (S2)
  d  stale latched QR goal          a goal stamped before the node started is ignored (F5)
  e  arrival                        ARRIVED -> IDLE and the commander ends in STOP
  f  nearby sequence waypoints      a waypoint inside the arrival box on selection wedges
                                    the commander without e3bb0e4 (S1, xfail) and advances
                                    with it

Run inside the container: ``bash demo/run_sm_test.sh`` (or pytest directly, with
``/opt/ros/jazzy`` and the workspace sourced). Without rclpy the whole module skips, so a
host-side ``pytest src/robot_mission_planner`` stays green.
"""

import json
import math
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS 2 (rclpy) is only available in the container")

from geographic_msgs.msg import GeoPointStamped  # noqa: E402
from geometry_msgs.msg import Pose, PoseArray, PoseStamped  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSDurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402

WS = Path(__file__).resolve().parents[3]
PKG = Path(__file__).resolve().parents[1]
DEMO = WS / "demo"
FAKE = DEMO / "fake_commander.py"
STATIC_TF = DEMO / "static_tf_2026-09-08.json"
CONFIG = PKG / "config" / "road_and_gps_follower.yaml"
MAP_FRAME = "FP_ENU0"
EARTH_FRAME = "FP_ECEF"
# One domain per scenario: the tests must not see each other, nor a bag replay on domain 0.
DOMAIN = {"a": 42, "b": 43, "c": 44, "d": 45, "e": 46, "f0": 47, "f1": 48, "g": 49}
SPEED = 3.0  # m/s of the simulated robot: keeps a 40 m scenario inside ~20 s

pytestmark = pytest.mark.skipif(
    not FAKE.exists() or shutil.which("ros2") is None,
    reason="needs demo/fake_commander.py and a sourced ROS 2 workspace",
)


# --------------------------------------------------------------------------- geometry
def quat_matrix(q):
    x, y, z, w = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def enu_to_ecef_transform():
    """(R, t) of the FP_ECEF -> FP_ENU0 transform the fake commander broadcasts."""
    entry = next(
        e
        for e in json.load(open(STATIC_TF))
        if e["parent"] == EARTH_FRAME and e["child"] == MAP_FRAME
    )
    return quat_matrix(entry["q"]), entry["t"]


def enu_to_latlon(xy, transform):
    """A point of our synthetic map-frame route as WGS84 (what a GPX/route carries)."""
    r, t = transform
    x, y, z = (sum(r[i][k] * v for k, v in enumerate((xy[0], xy[1], 0.0))) + t[i] for i in range(3))
    a, f = 6378137.0, 1.0 / 298.257223563
    b, e2 = a * (1 - f), f * (2 - f)
    ep2 = (a * a - b * b) / (b * b)
    p = math.hypot(x, y)
    th = math.atan2(a * z, b * p)
    lat = math.atan2(z + ep2 * b * math.sin(th) ** 3, p - e2 * a * math.cos(th) ** 3)
    return math.degrees(lat), math.degrees(math.atan2(y, x))


def densify(corners, spacing=3.0):
    """Polyline through ``corners`` with a point every ``spacing`` m (route waypoints)."""
    pts = [tuple(corners[0])]
    for a, b in zip(corners, corners[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(round(d / spacing)))
        for i in range(1, n + 1):
            pts.append((a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n))
    return pts


def write_gpx(path, route_enu, transform):
    body = "".join(
        f'<wpt lat="{lat:.9f}" lon="{lon:.9f}"><ele>0</ele><name>wp{i}</name></wpt>\n'
        for i, (lat, lon) in enumerate(enu_to_latlon(p, transform) for p in route_enu)
    )
    path.write_text(f'<?xml version="1.0"?>\n<gpx version="1.1" creator="test">\n{body}</gpx>\n')
    return path


def point_ahead(route, xy, ahead):
    """The point ``ahead`` metres along the route from the projection of ``xy`` on it."""
    best_i, best_t, best_d = 0, 0.0, float("inf")
    for i, (a, b) in enumerate(zip(route, route[1:])):
        vx, vy = b[0] - a[0], b[1] - a[1]
        den = vx * vx + vy * vy
        t = 0.0 if den == 0 else max(0.0, min(1.0, ((xy[0] - a[0]) * vx + (xy[1] - a[1]) * vy) / den))
        d = math.hypot(a[0] + vx * t - xy[0], a[1] + vy * t - xy[1])
        if d < best_d:
            best_i, best_t, best_d = i, t, d
    a, b = route[best_i], route[best_i + 1]
    seg = math.hypot(b[0] - a[0], b[1] - a[1])
    remaining = ahead - seg * (1.0 - best_t)
    if remaining <= 0:
        f = best_t + ahead / seg
        return a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f
    for a, b in zip(route[best_i + 1 :], route[best_i + 2 :]):
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if remaining <= seg:
            f = remaining / seg
            return a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f
        remaining -= seg
    return route[-1]


# --------------------------------------------------------------------------- rig
class Observer(Node):
    """Everything the follower needs that is neither the commander nor the route file."""

    def __init__(self, context, route, intersections=(), carrot_ahead=6.0):
        super().__init__("sm_observer", context=context)
        self.route = list(route)
        self.states, self.events, self.poses, self.status = [], [], [], []
        self.pose = None
        self.t0 = time.time()
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        history = QoSProfile(depth=200, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/road_follower/state", self._state_cb, 10)
        self.create_subscription(String, "/road_follower/event", self._event_cb, history)
        self.create_subscription(String, "/fake_commander/status", self._status_cb, 10)
        self.create_subscription(PoseStamped, "/fake_commander/pose", self._pose_cb, 10)
        self.pub_carrot = self.create_publisher(Marker, "/cloud_hull_center_marker", 10)
        self.pub_inter = self.create_publisher(PoseArray, "/intersections", latched)
        self.pub_qr = self.create_publisher(GeoPointStamped, "/qr_goal/goal", latched)
        self.carrot_ahead = carrot_ahead
        self.create_timer(0.1, self._publish_carrot)
        self._publish_intersections(intersections)

    # ---- observations
    def t(self):
        return time.time() - self.t0

    def _state_cb(self, msg):
        if not self.states or self.states[-1][1] != msg.data:
            self.states.append((round(self.t(), 1), msg.data, self.pose))

    def _event_cb(self, msg):
        self.events.append((round(self.t(), 1), msg.data))

    def _status_cb(self, msg):
        self.status.append(json.loads(msg.data))

    def _pose_cb(self, msg):
        yaw = 2 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
        self.pose = (msg.pose.position.x, msg.pose.position.y, yaw)
        self.poses.append((round(self.t(), 1),) + self.pose)

    # ---- stimuli
    def _publish_intersections(self, points):
        msg = PoseArray()
        msg.header.frame_id = MAP_FRAME
        for x, y in points:
            p = Pose()
            p.position.x, p.position.y = float(x), float(y)
            p.orientation.w = 1.0
            msg.poses.append(p)
        self.pub_inter.publish(msg)

    def _publish_carrot(self):
        """One road-centre point (build_point_cloud's SPHERE) ahead of the robot on the path."""
        if self.pose is None:
            return
        x, y = point_ahead(self.route, self.pose[:2], self.carrot_ahead)
        m = Marker()
        m.header.frame_id = MAP_FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.type, m.action = Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y = float(x), float(y)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.5
        self.pub_carrot.publish(m)

    def publish_qr_goal(self, lat, lon, age_s=0.0):
        msg = GeoPointStamped()
        stamp = self.get_clock().now().nanoseconds * 1e-9 - age_s
        msg.header.stamp.sec = int(stamp)
        msg.header.stamp.nanosec = int((stamp % 1) * 1e9)
        msg.position.latitude, msg.position.longitude = lat, lon
        self.pub_qr.publish(msg)

    # ---- helpers
    def state_text(self):
        return self.states[-1][1] if self.states else None

    def state_names(self):
        """State history with the fix-status suffix (``ROAD [rtk]``) and the like stripped."""
        return [s.split(" ")[0] for _, s, _ in self.states]


class PlanRouteServer:
    """
    Minimal stand-in for map_data's route_planner: hands back a fixed route, or fails
    every request with ``fail_reason`` (e.g. "snap_too_far") like the real node does.
    """

    def __init__(self, node, route_latlon, fail_reason=None):
        from map_data_interfaces.action import PlanRoute
        from rclpy.action import ActionServer

        self.type = PlanRoute
        self.route_latlon = route_latlon
        self.fail_reason = fail_reason
        self.calls = 0
        self.server = ActionServer(node, PlanRoute, "/route_planner/plan_route", self._execute)

    def _execute(self, goal_handle):
        from geographic_msgs.msg import GeoPoseStamped

        self.calls += 1
        result = self.type.Result()
        if self.fail_reason:
            result.success = False
            result.reason = self.fail_reason
            result.message = f"test planner: {self.fail_reason}"
            goal_handle.succeed()  # the action succeeds, the planning result says no
            return result
        result.success = True
        result.route.header.frame_id = "wgs84"
        for lat, lon in self.route_latlon:
            gp = GeoPoseStamped()
            gp.pose.position.latitude, gp.pose.position.longitude = lat, lon
            result.route.poses.append(gp)
        result.length_m = 3.0 * (len(self.route_latlon) - 1)
        result.message = "test route"
        goal_handle.succeed()
        return result


class Rig:
    """Fake commander + follower subprocesses and an in-process observer, on one domain."""

    def __init__(self, domain, route, tmp_path, fake_env=None, follower_params=None,
                 intersections=(), start_follower=True, plan_route=None, mission=False):
        self.domain = domain
        self.procs = []
        self.log_dir = tmp_path
        self.transform = enu_to_ecef_transform()
        self.route = list(route)
        self.route_latlon = [enu_to_latlon(p, self.transform) for p in self.route]
        env = {
            "FAKE_DRIVE": "1",
            "FAKE_SPEED": str(SPEED),
            "FAKE_SEQUENCE_SOURCE": "gpx",
            "STATIC_TF_JSON": str(STATIC_TF),
            "FAKE_EVENTS": str(tmp_path / "fake_events.jsonl"),
            "FAKE_START": f"{self.route[0][0]},{self.route[0][1]},0",
        }
        env.update(fake_env or {})
        self.events_path = Path(env["FAKE_EVENTS"])
        self.fake = self._spawn(["python3", str(FAKE)], env, "fake_commander.log")

        self.context = rclpy.Context()
        rclpy.init(context=self.context, domain_id=domain)
        self.observer = Observer(self.context, self.route, intersections)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.observer)
        # plan_route: True = a planner that answers with the route; a string = a planner
        # that fails every request with that reason.
        self.plan_route = (
            PlanRouteServer(
                self.observer, self.route_latlon,
                fail_reason=plan_route if isinstance(plan_route, str) else None,
            ) if plan_route else None
        )
        self.spin(1.0)

        self.gpx = write_gpx(tmp_path / "route.gpx", self.route, self.transform)
        # Mission mode = no `file` at all: rcl rejects an empty -p override value.
        params = {} if mission else {"file": str(self.gpx)}
        params.update(follower_params or {})
        if start_follower:
            self.start_follower(params)

    # ---- processes
    def _spawn(self, cmd, env, log):
        full = dict(os.environ)
        full.update(env)
        full["ROS_DOMAIN_ID"] = str(self.domain)
        full["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
        out = open(self.log_dir / log, "w")
        # Own process group: "ros2 run" does not forward SIGINT to the node it spawns, so
        # signalling the wrapper alone leaves an orphaned follower publishing latched goals
        # into the next test's domain.
        proc = subprocess.Popen(
            cmd, env=full, stdout=out, stderr=subprocess.STDOUT, cwd=str(WS), start_new_session=True
        )
        self.procs.append(proc)
        return proc

    def start_follower(self, params):
        args = ["ros2", "run", "robot_mission_planner", "road_follower", "--ros-args",
                "--params-file", str(CONFIG)]
        for k, v in params.items():
            args += ["-p", f"{k}:={v}"]
        self.follower = self._spawn(args, {}, "follower.log")
        return self.follower

    def stop(self):
        for sig, grace in ((signal.SIGINT, 3.0), (signal.SIGKILL, 3.0)):
            alive = [p for p in self.procs if p.poll() is None]
            if not alive:
                break
            for p in alive:
                try:
                    os.killpg(os.getpgid(p.pid), sig)
                except ProcessLookupError:
                    pass
            deadline = time.time() + grace
            for p in alive:
                try:
                    p.wait(timeout=max(0.1, deadline - time.time()))
                except subprocess.TimeoutExpired:
                    pass
        self.executor.remove_node(self.observer)
        self.observer.destroy_node()
        rclpy.shutdown(context=self.context)

    # ---- waiting
    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            self.executor.spin_once(timeout_sec=0.05)

    def wait_for(self, predicate, timeout, what=""):
        end = time.time() + timeout
        while time.time() < end:
            self.executor.spin_once(timeout_sec=0.05)
            if predicate():
                return True
        return False

    def require(self, predicate, timeout, what):
        assert self.wait_for(predicate, timeout, what), (
            f"timed out after {timeout} s waiting for {what}\n{self.report()}"
        )

    # ---- the fake commander's own record
    def fake_events(self):
        if not self.events_path.exists():
            return []
        return [json.loads(line) for line in self.events_path.read_text().splitlines() if line.strip()]

    def calls(self):
        """The service calls the follower made, in order: ('switch_mode', 'sequence'), ..."""
        out = []
        for e in self.fake_events():
            if e["event"] == "switch_mode":
                out.append((e["t"], "switch_mode", e["mode"]))
            elif e["event"] == "configure_sequence_mode":
                out.append((e["t"], "configure_sequence_mode", e["source"]))
            elif e["event"] == "goal_sequence":
                out.append((e["t"], "goal_sequence", e["poses"]))
        return out

    def report(self):
        return (
            f"follower states: {self.observer.states}\n"
            f"follower events: {self.observer.events}\n"
            f"commander calls: {self.calls()}\n"
            f"last commander status: {self.observer.status[-1] if self.observer.status else None}"
        )


@pytest.fixture
def rig(tmp_path):
    made = []

    def make(**kw):
        r = Rig(tmp_path=tmp_path, **kw)
        made.append(r)
        return r

    yield make
    for r in made:
        r.stop()


def wait_state(rig_, prefix, timeout, after=0):
    """Wait until the follower's state (index >= ``after``) starts with ``prefix``."""
    def hit():
        return any(s.startswith(prefix) for _, s, _ in rig_.observer.states[after:])

    rig_.require(hit, timeout, f"follower state {prefix}*")
    return next(i for i, (_, s, _) in enumerate(rig_.observer.states) if i >= after and s.startswith(prefix))


# --------------------------------------------------------------------------- scenarios
def gps_episode(r, node, radius=9.0):
    """
    The commander's record of the GPS episode at ``node``: from the ``goto`` that preceded
    the ring up to the ``goto`` that ends it. Anchored on the commander's own events (each
    carries the robot pose), so it needs no clock alignment with the follower and ignores
    any other GPS episode of the run.
    """
    def near(e):
        p = e.get("pose")
        return p is not None and math.hypot(p[0] - node[0], p[1] - node[1]) <= radius

    # The follower publishes its state right after deciding, i.e. a few ms before the
    # commander has served (and logged) the goto call that ends the episode: poll the
    # event file for a moment instead of reading it once.
    deadline = time.time() + 5.0
    while True:
        ev = r.fake_events()
        i_seq = next(
            (i for i, e in enumerate(ev)
             if e["event"] == "switch_mode" and e["mode"] == "sequence" and near(e)),
            None,
        )
        i_end = None
        if i_seq is not None:
            i_end = next(
                (i for i in range(i_seq + 1, len(ev))
                 if ev[i]["event"] == "switch_mode" and ev[i]["mode"] == "goto"),
                None,
            )
        if i_end is not None or time.time() > deadline:
            break
        time.sleep(0.2)
    assert i_seq is not None, f"no sequence switch near {node}\n{r.report()}"
    assert i_end is not None, f"no goto switch after the sequence near {node}\n{r.report()}"
    i_start = max(
        (i for i in range(i_seq) if ev[i]["event"] == "switch_mode" and ev[i]["mode"] == "goto"),
        default=0,
    )
    return ev[i_start : i_end + 1]


def call_kinds(events):
    """The follower's service calls / publications in an episode, in order."""
    out = []
    for e in events:
        if e["event"] == "switch_mode":
            out.append(("switch_mode", e["mode"]))
        elif e["event"] == "configure_sequence_mode":
            out.append(("configure_sequence_mode", e["source"]))
        elif e["event"] == "goal_sequence":
            out.append(("goal_sequence", e["poses"]))
    return out


def test_a_straight_route_one_ring_without_stopping(rig):
    """ROAD -> GPS:intersection -> ROAD with no STOP in between and one sequence per entry."""
    # 60 m: the ring sits at 24 m and the follower is back in ROAD well before the final
    # approach (the last ~15 m of a route are driven in GPS).
    route = densify([(0, 0), (60, 0)])
    r = rig(domain=DOMAIN["a"], route=route, intersections=[(24.0, 0.0)])
    i_road = wait_state(r, "ROAD", 30)
    i_gps = wait_state(r, "GPS:intersection", 40, after=i_road + 1)
    i_back = wait_state(r, "ROAD", 40, after=i_gps + 1)

    t_gps, t_back = r.observer.states[i_gps][0], r.observer.states[i_back][0]
    window = call_kinds(gps_episode(r, (24.0, 0.0)))
    # F1: the hand-over is goto <-> sequence directly; a STOP here costs seconds per ring.
    assert ("switch_mode", "stop") not in window, f"STOP in the hand-over: {window}\n{r.report()}"
    assert window[0] == ("switch_mode", "goto"), window
    # F2: one whole-route sequence per GPS entry, not one per 10-waypoint window.
    assert sum(1 for k in window if k[0] == "goal_sequence") == 1, f"{window}\n{r.report()}"
    # S2: the source is configured before every sequence, not once per process.
    assert sum(1 for k in window if k[0] == "configure_sequence_mode") == 1, window
    assert t_back - t_gps < 30, "spent too long at one ring"
    assert r.observer.pose[0] > 24.0, "the robot never passed the intersection"


def test_b_right_angle_junction_is_left_after_the_node(rig):
    """F3: the exit test uses the route direction *after* the node, so a 90 deg turn exits."""
    route = densify([(0, 0), (24, 0), (24, 36)])
    r = rig(domain=DOMAIN["b"], route=route, intersections=[(24.0, 0.0)])
    i_road = wait_state(r, "ROAD", 30)
    i_gps = wait_state(r, "GPS:intersection", 40, after=i_road + 1)
    wait_state(r, "ROAD", 60, after=i_gps + 1)

    # Where the hand-over happened, as the commander saw it. The follower's own state topic
    # is published at the top of its 1 Hz tick, i.e. one tick *after* the transition.
    episode = gps_episode(r, (24.0, 0.0))
    pose = episode[-1].get("pose")
    assert pose is not None, r.report()
    # Distance travelled past the junction along the outgoing (northbound) leg.
    past = math.hypot(pose[0] - 24.0, pose[1] - 0.0)
    assert pose[1] > 0.0, f"left GPS before the turn ({pose})\n{r.report()}"
    assert past < 10.0, f"still in GPS {past:.1f} m after the node\n{r.report()}"
    assert ("switch_mode", "stop") not in call_kinds(episode), r.report()


def test_c_commander_restart_mid_gps_is_recovered(rig):
    """S2: a restarted commander is back on source=gpx; the follower must re-configure."""
    route = densify([(0, 0), (60, 0)])
    r = rig(
        domain=DOMAIN["c"],
        route=route,
        intersections=[(0.0, 0.0)],  # in the ring from the first tick: the whole run is GPS
        fake_env={"FAKE_RESTART_AT": "20", "FAKE_SPEED": "1.5"},
        # Stay in GPS for the whole scenario so the restart cannot land in ROAD mode.
        follower_params={"intersection_enter_threshold": 8.0, "intersection_exit_threshold": 200.0},
    )
    wait_state(r, "GPS:intersection", 30)
    r.require(lambda: any(e["event"] == "restart" for e in r.fake_events()), 40, "the fake restart")
    t_restart = next(e["t"] for e in r.fake_events() if e["event"] == "restart")
    x_restart = r.observer.pose[0]

    # The follower must notice the state-topic gap, re-send configure(topic) + the sequence.
    r.require(
        lambda: any(
            e["event"] == "sequence_loaded" and e["source"] == "topic" and e["t"] > t_restart
            for e in r.fake_events()
        ),
        40,
        "the sequence to be reloaded from the topic after the restart",
    )
    events = r.fake_events()
    reconfig = next(
        e for e in events
        if e["event"] == "configure_sequence_mode" and e["source"] == "topic" and e["t"] > t_restart
    )
    reloaded = next(
        e for e in events
        if e["event"] == "sequence_loaded" and e["source"] == "topic" and e["t"] > t_restart
    )
    gpx_errors = [e["t"] for e in events if e["event"] == "gpx_source_error" and e["t"] > t_restart]
    if gpx_errors:
        assert reloaded["t"] - gpx_errors[0] < 10.0, (
            f"stuck on the gpx sequence source for {reloaded['t'] - gpx_errors[0]:.0f} s\n{r.report()}"
        )
    assert reconfig["t"] < reloaded["t"]
    assert any(e["event"] == "goal_sequence" and e["t"] > t_restart for e in events), r.report()
    r.require(lambda: r.observer.pose[0] > x_restart + 5.0, 30, "the robot to drive again")


def test_d_stale_latched_qr_goal_is_ignored(rig):
    """F5: a restarted follower must not re-drive the previous run's latched QR goal."""
    route = densify([(0, 0), (30, 0)])
    r = rig(domain=DOMAIN["d"], route=route, start_follower=False, mission=True)
    lat, lon = r.route_latlon[-1]
    r.observer.publish_qr_goal(lat, lon, age_s=60.0)
    r.spin(1.0)
    r.start_follower({})

    r.require(lambda: r.observer.state_text() is not None, 30, "the follower to publish a state")
    r.spin(12.0)
    assert set(r.observer.state_names()) == {"IDLE"}, r.report()
    assert not [e for _, e in r.observer.events if e.startswith(("GOAL", "PLANNING", "ROUTE"))], r.report()
    assert not [c for c in r.calls() if c[1] != "switch_mode" or c[2] != "stop"], r.report()


def test_e_arrival_stops_the_commander(rig):
    """The mission ends: ARRIVED -> IDLE, and the commander is put back into STOP."""
    route = densify([(0, 0), (36, 0)])
    r = rig(
        domain=DOMAIN["e"],
        route=route,
        mission=True,
        plan_route=True,
        start_follower=False,
    )
    r.start_follower({"start_delay": 2.0})
    r.require(lambda: (r.observer.state_text() or "").startswith("IDLE"), 30, "IDLE")
    lat, lon = r.route_latlon[-1]
    r.observer.publish_qr_goal(lat, lon)

    wait_state(r, "PLANNING", 20)
    wait_state(r, "ROAD", 30)
    i_arrived = wait_state(r, "ARRIVED", 60)
    wait_state(r, "IDLE", 20, after=i_arrived + 1)
    assert r.plan_route.calls == 1
    r.require(lambda: any(e.startswith("IDLE") for _, e in r.observer.events), 10, "the IDLE event")
    # The mission events in order; the node may publish others in between (HOME, FIX, ...).
    names = [e.split(":")[0] for _, e in r.observer.events]
    expected = ["GOAL", "PLANNING", "ROUTE", "START", "ARRIVED", "IDLE"]
    it = iter(names)
    assert all(any(n == want for n in it) for want in expected), f"{names}\n{r.report()}"
    r.require(
        lambda: r.observer.status and r.observer.status[-1]["mode"] == "STOP", 15, "commander STOP"
    )
    assert r.observer.pose[0] > 30.0, r.report()


@pytest.mark.parametrize(
    "nearby_fix",
    [
        pytest.param(
            "0",
            marks=pytest.mark.xfail(
                strict=False,
                reason="S1: without e3bb0e4 a sequence waypoint inside the arrival box on "
                "selection is consumed without advancing the index and the commander wedges",
            ),
        ),
        "1",
    ],
    ids=["without-e3bb0e4", "with-e3bb0e4"],
)
def test_f_sequence_waypoint_inside_the_arrival_box(rig, nearby_fix):
    """
    S1 regression, commander-side only (no follower): two waypoints 1 m apart.

    ``findInitialPoint`` starts at nearest + 1, which lands inside the 1.5 x 2.5 m arrival
    box, and ``updateGoal`` consumes a goal it is already standing on. Only e3bb0e4 turns
    that into a ``handleGoalReached()`` and advances the index; without it
    ``refreshSequenceGoal`` re-proposes the same waypoint forever.
    """
    route = [(0.0, 0.0), (1.0, 0.0), (4.0, 0.0), (7.0, 0.0), (10.0, 0.0), (13.0, 0.0)]
    r = rig(
        domain=DOMAIN["f" + nearby_fix],
        route=route,
        start_follower=False,
        fake_env={"FAKE_NEARBY_FIX": nearby_fix, "FAKE_SEQUENCE_SOURCE": "topic"},
    )
    from crl_commander.srv import ConfigureSequenceMode, SwitchMode

    node = r.observer
    latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(PoseArray, "/goal_sequence", latched)
    switch = node.create_client(SwitchMode, "/crl_commander/switch_mode")
    configure = node.create_client(ConfigureSequenceMode, "/crl_commander/configure_sequence_mode")
    r.require(lambda: switch.service_is_ready() and configure.service_is_ready(), 20, "the services")

    seq = PoseArray()
    seq.header.frame_id = EARTH_FRAME
    rot, trans = r.transform
    for x, y in route:
        p = Pose()
        p.position.x, p.position.y, p.position.z = (
            sum(rot[i][k] * v for k, v in enumerate((x, y, 0.0))) + trans[i] for i in range(3)
        )
        p.orientation.w = 1.0
        seq.poses.append(p)
    pub.publish(seq)
    req = ConfigureSequenceMode.Request()
    req.source, req.loop = "topic", False
    configure.call_async(req)
    req = SwitchMode.Request()
    req.mode = "sequence"
    switch.call_async(req)

    r.require(lambda: any(e["event"] == "sequence_loaded" for e in r.fake_events()), 20, "the sequence")
    assert next(e for e in r.fake_events() if e["event"] == "sequence_loaded")["start"] == 1
    # With the fix the commander is past the nearby waypoint and driving within a second or
    # two; without it, it consumes waypoint 1 forever and the robot never moves.
    moved = r.wait_for(lambda: r.observer.pose is not None and r.observer.pose[0] > 5.0, 8)
    consumed = [e for e in r.fake_events() if e["event"] == "goal_consumed"]
    indices = [s["seq_index"] for s in r.observer.status]
    assert moved, (
        f"the commander never left waypoint 1: indices={sorted(set(indices))}, "
        f"{len(consumed)} consume events, pose={r.observer.pose}"
    )
    assert max(indices) > 1


def test_g_unsnappable_goal_is_not_retried(rig):
    """
    A goal the planner cannot snap (snap_too_far: farther from any way than its limit)
    is dropped after one attempt, without the plan_retries x plan_retry_delay wait, and
    the follower is back in IDLE with a PLAN_FAILED event.
    """
    route = densify([(0, 0), (36, 0)])
    r = rig(
        domain=DOMAIN["g"],
        route=route,
        mission=True,
        plan_route="snap_too_far",
        start_follower=False,
    )
    r.start_follower({"plan_retries": 3, "plan_retry_delay": 20.0})
    r.require(lambda: (r.observer.state_text() or "").startswith("IDLE"), 30, "IDLE")
    lat, lon = r.route_latlon[-1]
    r.observer.publish_qr_goal(lat, lon)

    i_planning = wait_state(r, "PLANNING", 20)
    # Well inside one plan_retry_delay: a retry would still be pending.
    wait_state(r, "IDLE", 10, after=i_planning + 1)
    assert r.plan_route.calls == 1, r.report()
    failed = [e for _, e in r.observer.events if e.startswith("PLAN_FAILED")]
    assert failed and "snap_too_far" in failed[0], f"{r.observer.events}\n{r.report()}"
    names = [s for s in r.observer.state_names()]
    assert "ROAD" not in names and "GPS" not in names, r.report()

