"""
Road-goal selection for the ROAD state of ``road_follower``.

Pure geometry, no ROS imports, so it can be unit-tested without a ROS
installation. Everything is in the follower's ``map_frame`` (x, y in metres).

The commander (``crl_commander``) treats a goal that is already inside its
arrival box (``goal_reached_dist_x/y``, 2.5 m on Helhest) as *reached* and
holds position instead of driving to it. Every selector below therefore
guarantees that the returned goal is at least ``min_ahead`` metres away from the
robot: a closer input is pushed outwards along its own bearing, so the robot
keeps moving in the direction the road perception points to.

Three selectors, one per ``road_goal_source``: :func:`select_carrot_goal` (one road-centre
point), :func:`select_path_goal` (a predicted road path) and :func:`select_route_goal`, which
combines the carrot with the *shape* of the planned OSM route -- see its docstring for why the
route is only ever used relative to the robot's own projection on it.
"""

import bisect
import math

Point = tuple[float, float]
Goal = tuple[float, float, float]  # x, y, yaw


def _bearing(from_xy: Point, to_xy: Point) -> float:
    return math.atan2(to_xy[1] - from_xy[1], to_xy[0] - from_xy[0])


def _dist(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _push_out(
    robot_xy: Point, target_xy: Point, distance: float, fallback_yaw: float
) -> Goal:
    """Point ``distance`` metres from the robot in the direction of ``target_xy``."""
    d = _dist(robot_xy, target_xy)
    yaw = _bearing(robot_xy, target_xy) if d > 1e-6 else fallback_yaw
    return (
        robot_xy[0] + distance * math.cos(yaw),
        robot_xy[1] + distance * math.sin(yaw),
        yaw,
    )


def select_carrot_goal(
    carrot_xy: Point,
    robot_xy: Point,
    robot_yaw: float,
    min_ahead: float,
    max_ahead: float,
) -> Goal | None:
    """
    Turn a single road-centre point (the convex-hull centre of the road points
    in the current lidar frame) into a commander goal.

    Returns ``None`` when the carrot is farther than ``max_ahead`` (beyond the
    sensor range it can only be a projection artefact). A carrot closer than
    ``min_ahead`` is pushed out to ``min_ahead`` along the robot -> carrot
    bearing (robot heading if the carrot sits on the robot).
    """
    d = _dist(robot_xy, carrot_xy)
    if d > max_ahead:
        return None
    if d < min_ahead:
        return _push_out(robot_xy, carrot_xy, min_ahead, robot_yaw)
    return carrot_xy[0], carrot_xy[1], _bearing(robot_xy, carrot_xy)


def select_path_goal(
    path_xy: list[Point],
    robot_xy: Point,
    robot_yaw: float,
    min_ahead: float,
    max_ahead: float,
) -> Goal | None:
    """
    Pick the commander goal from a predicted road path (``/predicted_path_ls``).

    The goal is the *last* path point that is at most ``max_ahead`` from the
    robot. When that point is closer than ``min_ahead`` the path is
    extrapolated: along its final segment if it has one, otherwise along the
    robot -> point bearing, until the goal is ``min_ahead`` away. An empty path
    or a path whose every point is beyond ``max_ahead`` gives ``None``.
    """
    if not path_xy:
        return None
    within = [p for p in path_xy if _dist(robot_xy, p) <= max_ahead]
    if not within:
        return None
    end = within[-1]
    if _dist(robot_xy, end) >= min_ahead:
        prev = path_xy[path_xy.index(end) - 1] if path_xy.index(end) > 0 else robot_xy
        yaw = (
            _bearing(prev, end) if _dist(prev, end) > 1e-6 else _bearing(robot_xy, end)
        )
        return end[0], end[1], yaw

    # Extrapolate along the path's own direction when it has one that leads
    # away from the robot; a path curling back towards the robot is not
    # continued (that would place the goal behind it).
    idx = path_xy.index(end)
    if idx > 0 and _dist(path_xy[idx - 1], end) > 1e-6:
        yaw = _bearing(path_xy[idx - 1], end)
        rel = (end[0] - robot_xy[0], end[1] - robot_xy[1])
        if rel[0] * math.cos(yaw) + rel[1] * math.sin(yaw) > 0.0:
            step = 0.25
            x, y = end
            for _ in range(int(4 * max_ahead / step)):
                if _dist(robot_xy, (x, y)) >= min_ahead:
                    return x, y, yaw
                x += step * math.cos(yaw)
                y += step * math.sin(yaw)
    return _push_out(robot_xy, end, min_ahead, robot_yaw)


def polyline_cumulative(points: list[Point]) -> list[float]:
    """Cumulative arclength (m) at every vertex of the polyline."""
    cum = [0.0]
    for a, b in zip(points, points[1:]):
        cum.append(cum[-1] + _dist(a, b))
    return cum


def _segment_range(count: int, index: int, window: int) -> range:
    """Segment indices searched around waypoint ``index`` (``window <= 0`` = all of them)."""
    if count < 1:
        return range(0)
    if window <= 0:
        return range(count)
    lo = max(0, min(index, count - 1) - window)
    hi = min(count - 1, index + window)
    return range(lo, hi + 1)


def project_on_route(
    points: list[Point], cum: list[float], xy: Point, index: int = 0, window: int = 0
) -> tuple[float, float] | None:
    """
    Project ``xy`` on the route polyline: ``(arclength, signed lateral offset)``, the offset
    positive to the left of the direction of travel.

    Only segments within ``window`` waypoints of ``index`` are searched, so a route that comes
    back close to itself (Stromovka legs run parallel 10 m apart) is projected onto the leg the
    robot is actually driving. ``None`` for a polyline with fewer than two points.
    """
    if len(points) < 2 or len(cum) != len(points):
        return None
    best_d, best = float("inf"), None
    for i in _segment_range(len(points) - 1, index, window):
        a, b = points[i], points[i + 1]
        abx, aby = b[0] - a[0], b[1] - a[1]
        seg = math.hypot(abx, aby)
        if seg <= 1e-9:
            continue
        apx, apy = xy[0] - a[0], xy[1] - a[1]
        t = min(1.0, max(0.0, (apx * abx + apy * aby) / (seg * seg)))
        d = math.hypot(apx - t * abx, apy - t * aby)
        if d < best_d:
            best_d = d
            best = (cum[i] + t * seg, (abx * apy - aby * apx) / seg)
    return best


def route_point_at(
    points: list[Point], cum: list[float], s: float
) -> tuple[Point, float]:
    """
    ``((x, y), yaw)`` at arclength ``s`` along the polyline. ``s`` outside the route is
    extrapolated along the first / last segment: the goal may have to be pushed past the end of
    a short route to stay outside the commander's arrival box (the follower's final approach
    normally takes over long before that).

    Exactly on a vertex the *incoming* segment gives the heading (and with it the normal the
    lateral offset is applied along), so a goal clamped at a corner still faces the way the
    robot approaches it.
    """
    if len(points) < 2:
        return (points[0] if points else (0.0, 0.0)), 0.0
    i = min(max(bisect.bisect_left(cum, s) - 1, 0), len(points) - 2)
    a, b = points[i], points[i + 1]
    seg = cum[i + 1] - cum[i]
    if seg <= 1e-9:
        return a, 0.0
    t = (s - cum[i]) / seg
    return (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])), _bearing(a, b)


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def turn_limited_arclength(
    points: list[Point], cum: list[float], s_from: float, s_to: float, max_turn: float
) -> float:
    """
    Cut ``s_to`` back to the first route vertex whose segment heading differs from the heading
    at ``s_from`` by more than ``max_turn`` (rad): the goal must not sit around a corner, where
    the local planner cannot see and the road perception does not reach. ``max_turn <= 0`` = no
    limit.
    """
    if max_turn <= 0.0 or s_to <= s_from or len(points) < 2:
        return s_to
    # The reference is the heading the robot leaves ``s_from`` with, so a corner it is already
    # standing on does not clamp the stretch to zero.
    first = min(max(bisect.bisect_right(cum, s_from) - 1, 0), len(points) - 2)
    yaw0 = _bearing(points[first], points[first + 1])
    for i in range(first + 1, len(points) - 1):
        if cum[i] >= s_to:
            break
        if abs(_wrap(_bearing(points[i], points[i + 1]) - yaw0)) > max_turn:
            return max(s_from, cum[i])
    return s_to


def select_route_goal(
    carrot_xy: Point | None,
    robot_xy: Point,
    points: list[Point],
    cum: list[float],
    index: int,
    stretch: float,
    min_ahead: float,
    max_ahead: float,
    lateral_limit: float,
    lateral_gain: float = 1.0,
    max_turn: float = 0.0,
    window: int = 0,
) -> Goal | None:
    """
    Road goal from the *shape* of the planned OSM route, placed by the road perception.

    The route is used relatively, never as an absolute position: both the robot and the carrot
    (the convex-hull centre of the road points) are projected onto it, and the goal is put
    ``stretch`` metres further along the route from whichever of the two is ahead, carrying the
    carrot's own lateral offset (``lateral_gain`` of it, clamped to ``lateral_limit``, 0 = no
    clamp) over to that point. So the map contributes the direction the road takes -- which a
    hull centre lagging behind the robot cannot supply -- while the offset between the map and
    the real drivable surface (GNSS error plus OSM error, up to ~5 m under trees) is measured
    by the lidar every frame and reproduced at the goal.

    The result is kept at least ``min_ahead`` from the robot (a nearer goal sits inside the
    commander's arrival box and stops it, which wins over every other limit here) and, if that
    allows, at most ``max_ahead`` from it and before the first corner sharper than ``max_turn``.
    ``None`` when there is no usable route, or when even the start of the stretch is farther
    than ``max_ahead`` (the projection cannot be trusted then).
    """
    proj_r = project_on_route(points, cum, robot_xy, index, window)
    if proj_r is None:
        return None
    s_robot, lateral_robot = proj_r
    s_carrot, lateral = s_robot, lateral_robot
    if carrot_xy is not None:
        proj_c = project_on_route(points, cum, carrot_xy, index, window)
        if proj_c is None:
            return None
        s_carrot, lateral = proj_c
    lateral *= lateral_gain
    if lateral_limit > 0.0:
        lateral = max(-lateral_limit, min(lateral_limit, lateral))

    def goal_at(s: float) -> Goal:
        (x, y), yaw = route_point_at(points, cum, s)
        return x - lateral * math.sin(yaw), y + lateral * math.cos(yaw), yaw

    # The carrot never pulls the goal back: a hull centre that lags the robot is not evidence
    # that the road ends there.
    s_base = max(s_robot, s_carrot)
    s_goal = turn_limited_arclength(
        points, cum, s_base, s_base + max(0.0, stretch), max_turn
    )

    if max_ahead > 0.0 and _dist(robot_xy, goal_at(s_goal)[:2]) > max_ahead:
        lo, hi = min(s_robot, s_goal), s_goal
        if _dist(robot_xy, goal_at(lo)[:2]) > max_ahead:
            return (
                None  # the route itself is out of reach: a bad projection, not a goal
            )
        for _ in range(20):
            mid = 0.5 * (lo + hi)
            if _dist(robot_xy, goal_at(mid)[:2]) > max_ahead:
                hi = mid
            else:
                lo = mid
        s_goal = lo

    step = 0.5
    for _ in range(int(2.0 * (min_ahead + max(max_ahead, min_ahead)) / step) + 4):
        if _dist(robot_xy, goal_at(s_goal)[:2]) >= min_ahead:
            return goal_at(s_goal)
        s_goal += step
    return None


def is_behind(goal_xy: Point, robot_xy: Point, robot_yaw: float) -> bool:
    """True when the goal lies in the half-plane behind the robot."""
    dx, dy = goal_xy[0] - robot_xy[0], goal_xy[1] - robot_xy[1]
    return dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw) < 0.0


def smooth(previous: Point | None, current: Point, alpha: float) -> Point:
    """Exponential smoothing; ``alpha`` 0 = raw, 0.9 = heavy."""
    if previous is None or alpha <= 0.0:
        return current
    a = min(alpha, 0.99)
    return (
        a * previous[0] + (1.0 - a) * current[0],
        a * previous[1] + (1.0 - a) * current[1],
    )


def is_arrived(
    robot_xy: Point,
    waypoints_xy: list[Point | None],
    current_index: int,
    radius: float,
    index_window: int = 3,
) -> bool:
    """
    True when the robot is within ``radius`` of the last waypoint and the follower's
    index is within ``index_window`` waypoints of the end. The index guard stops a
    route that starts next to its own goal (or loops back past it) from finishing
    before it started.
    """
    pts = [p for p in waypoints_xy if p is not None]
    if not pts:
        return False
    if current_index < len(waypoints_xy) - 1 - max(0, index_window):
        return False
    return _dist(robot_xy, pts[-1]) <= radius


def remaining_route_length(
    robot_xy: Point, waypoints_xy: list[Point | None], current_index: int
) -> float:
    """
    Route length still ahead: robot -> waypoint ``current_index`` -> ... -> last waypoint.

    Used for the final approach (the follower stays in GPS for the last few metres, where
    the route leaves the footway towards the goal itself). ``None`` entries (waypoints
    without a map transform yet) are skipped and an empty route gives ``inf``, so a caller
    comparing against a threshold never triggers on missing data.
    """
    if not waypoints_xy:
        return float("inf")
    idx = max(0, min(int(current_index), len(waypoints_xy) - 1))
    pts = [p for p in waypoints_xy[idx:] if p is not None]
    if not pts:
        return float("inf")
    total = _dist(robot_xy, pts[0])
    for a, b in zip(pts, pts[1:]):
        total += _dist(a, b)
    return total


def latlon_distance(a: Point, b: Point) -> float:
    """
    Approximate distance (m) between two ``(lat, lon)`` pairs (equirectangular, the mean
    latitude for the longitude scale). Well under a percent of error over a few kilometres,
    which is all the follower asks of it ("is this the goal we are already driving to?").
    """
    lat_m = (a[0] - b[0]) * 111320.0
    lon_m = (a[1] - b[1]) * 111320.0 * math.cos(math.radians((a[0] + b[0]) / 2.0))
    return math.hypot(lat_m, lon_m)


def nearest_index(points_xy: list[Point | None], xy: Point) -> int:
    """Index of the point closest to ``xy`` (``None`` entries skipped; 0 if none)."""
    best, best_d = 0, float("inf")
    for i, p in enumerate(points_xy):
        if p is None:
            continue
        d = _dist(p, xy)
        if d < best_d:
            best, best_d = i, d
    return best


def passed_along(robot_xy: Point, node_xy: Point, direction) -> bool:
    """
    True when the robot is beyond ``node_xy`` along ``direction`` (a unit vector or
    ``None``). With no direction there is nothing to test, so the node counts as passed.
    """
    if direction is None:
        return True
    rel = (robot_xy[0] - node_xy[0], robot_xy[1] - node_xy[1])
    return rel[0] * float(direction[0]) + rel[1] * float(direction[1]) > 0.0


def route_offset_limit(
    robot_offset: float | None, base_limit: float, margin: float, hard_limit: float
) -> float:
    """
    How far off the planned route a road goal may be: at least ``base_limit``, or the
    robot's own offset plus ``margin`` when the robot is already farther off the OSM line
    than that (a correct carrot 2 m ahead on the real path sits next to the robot, not
    on the map centreline), capped at ``hard_limit`` (``<= 0`` = no cap).
    """
    limit = base_limit
    if robot_offset is not None and math.isfinite(robot_offset):
        limit = max(limit, robot_offset + margin)
    if hard_limit > 0:
        limit = min(limit, max(hard_limit, base_limit))
    return limit


def _point_segment_distance(p: Point, a: Point, b: Point) -> float:
    """Distance from ``p`` to the segment ``a`` -> ``b`` (to ``a`` for a degenerate one)."""
    abx, aby = b[0] - a[0], b[1] - a[1]
    denom = abx * abx + aby * aby
    if denom <= 0.0:
        return _dist(p, a)
    t = ((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / denom
    t = min(1.0, max(0.0, t))
    return math.hypot(p[0] - (a[0] + t * abx), p[1] - (a[1] + t * aby))


def indices_near_polyline(
    points: list[Point], polyline: list[Point], max_distance: float
) -> list[int]:
    """
    Indices of the points that lie at most ``max_distance`` from ``polyline`` (its
    vertices, in order).

    Used to keep only the OSM intersections that sit on the planned route: a ring on a
    side junction the route merely drives past is not ours. ``max_distance <= 0`` or a
    polyline with fewer than two vertices means "no filter": every index is returned.
    """
    if max_distance <= 0.0 or len(polyline) < 2:
        return list(range(len(points)))
    kept = []
    for i, p in enumerate(points):
        for a, b in zip(polyline, polyline[1:]):
            # Cheap bounding-box reject first: a route has many segments, and all but a
            # few are nowhere near the point.
            if (
                not min(a[0], b[0]) - max_distance
                <= p[0]
                <= max(a[0], b[0]) + max_distance
            ):
                continue
            if (
                not min(a[1], b[1]) - max_distance
                <= p[1]
                <= max(a[1], b[1]) + max_distance
            ):
                continue
            if _point_segment_distance(p, a, b) <= max_distance:
                kept.append(i)
                break
    return kept
