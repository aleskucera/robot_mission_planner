"""
Route shift for ``mode: gps_shift``: how far the planned route is off the road the robot sees.

The route is driven as GPS waypoints all the way; the road segmentation only moves them.
``road_map_2`` (build_map's grid of road cells in ``map_frame``; its ``cost`` field is the
centerline cost image / 255, 0 at the road centre up to ~0.99 at its edge) is cropped to a circle around the robot, the centre band is kept, and
the route polyline is translated -- never rotated -- onto those cells. That absorbs both a
misplaced OSM way and a GNSS offset.

On a straight road only the lateral part of the shift is measured: the residual from a point
to its closest point on a straight segment is perpendicular to the segment, so nothing slides
the route along the road. Where the route bends inside the circle both components come out.

Pure numpy, no ROS imports, so it can be unit-tested without a ROS installation.
"""

import numpy as np


def centre_points(xy, cost, robot_xy, radius: float, max_cost: float) -> np.ndarray:
    """The (N, 2) road-centre cells (``cost <= max_cost``) within ``radius`` of the robot."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    near = np.hypot(xy[:, 0] - robot_xy[0], xy[:, 1] - robot_xy[1]) <= radius
    return xy[near & (np.asarray(cost).reshape(-1) <= max_cost)]


def _closest_on_segments(
    points: np.ndarray, a: np.ndarray, b: np.ndarray
) -> np.ndarray:
    """(N, S, 2): the closest point of every segment ``a[s]`` -> ``b[s]`` to every point."""
    ab = b - a
    t = np.einsum("nsk,sk->ns", points[:, None, :] - a[None], ab)
    t = np.clip(t / np.maximum(np.einsum("sk,sk->s", ab, ab), 1e-12), 0.0, 1.0)
    return a[None] + t[..., None] * ab[None]


def fit_route_shift(
    centre: np.ndarray,
    polyline,
    robot_xy,
    radius: float,
    max_shift: float,
    min_points: int,
    iterations: int = 5,
) -> tuple[float, float] | None:
    """
    Translation ``(dx, dy)`` that moves the route onto the road-centre points: a
    translation-only ICP (closest route point per centre point, mean residual, repeat).

    Only the route segments within ``radius + max_shift`` of the robot take part, so the rest
    of the leg cannot attract the points. ``None`` with fewer than ``min_points`` centre points
    or no route segment nearby; a longer shift is clamped to ``max_shift``.
    """
    pts = np.asarray(polyline, dtype=float).reshape(-1, 2)
    if len(centre) < max(1, min_points) or len(pts) < 2:
        return None
    a, b = pts[:-1], pts[1:]
    robot = np.asarray(robot_xy, dtype=float).reshape(1, 2)
    to_robot = np.hypot(*(robot[:, None, :] - _closest_on_segments(robot, a, b))[0].T)
    near = to_robot <= radius + max_shift
    if not near.any():
        return None
    a, b = a[near], b[near]
    centre = np.asarray(centre, dtype=float)
    rows = np.arange(len(centre))
    shift = np.zeros(2)
    for _ in range(iterations):
        closest = _closest_on_segments(centre, a + shift, b + shift)
        d2 = np.sum((centre[:, None, :] - closest) ** 2, axis=2)
        step = np.mean(centre - closest[rows, d2.argmin(axis=1)], axis=0)
        shift += step
        if np.hypot(*step) < 1e-3:
            break
    norm = float(np.hypot(*shift))
    if max_shift > 0 and norm > max_shift:
        shift *= max_shift / norm
    return float(shift[0]), float(shift[1])
