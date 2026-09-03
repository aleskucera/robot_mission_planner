"""
Road-goal selection for the ROAD state of ``road_follower``.

Pure geometry, no ROS imports, so it can be unit-tested without a ROS
installation. Everything is in the follower's ``map_frame`` (x, y in metres).

The commander (``crl_commander``) treats a goal that is already inside its
arrival box (``goal_reached_dist_x/y``, 2.5 m on Helhest) as *reached* and
holds position instead of driving to it. Both selectors below therefore
guarantee that the returned goal is at least ``min_ahead`` metres away from the
robot: a closer input is pushed outwards along its own bearing, so the robot
keeps moving in the direction the road perception points to.
"""

import math

Point = tuple[float, float]
Goal = tuple[float, float, float]  # x, y, yaw


def _bearing(from_xy: Point, to_xy: Point) -> float:
    return math.atan2(to_xy[1] - from_xy[1], to_xy[0] - from_xy[0])


def _dist(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _push_out(robot_xy: Point, target_xy: Point, distance: float, fallback_yaw: float) -> Goal:
    """Point ``distance`` metres from the robot in the direction of ``target_xy``."""
    d = _dist(robot_xy, target_xy)
    yaw = _bearing(robot_xy, target_xy) if d > 1e-6 else fallback_yaw
    return robot_xy[0] + distance * math.cos(yaw), robot_xy[1] + distance * math.sin(yaw), yaw


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
        yaw = _bearing(prev, end) if _dist(prev, end) > 1e-6 else _bearing(robot_xy, end)
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


def is_behind(goal_xy: Point, robot_xy: Point, robot_yaw: float) -> bool:
    """True when the goal lies in the half-plane behind the robot."""
    dx, dy = goal_xy[0] - robot_xy[0], goal_xy[1] - robot_xy[1]
    return dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw) < 0.0


def smooth(previous: Point | None, current: Point, alpha: float) -> Point:
    """Exponential smoothing; ``alpha`` 0 = raw, 0.9 = heavy."""
    if previous is None or alpha <= 0.0:
        return current
    a = min(alpha, 0.99)
    return a * previous[0] + (1.0 - a) * current[0], a * previous[1] + (1.0 - a) * current[1]
