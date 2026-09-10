"""The route the follower drives: waypoints, where they are, and how far along them we are.

A route is a list of ``{lat, lon, ele}`` points, from a GPX/YAML file or from a
``PlanRoute`` answer (the Robotour mission). Everything derived from it lives here: the
waypoints in ``map_frame``, the polyline (segments and cumulative arclength) the road goal
and the offset checks are measured against, and the index of the waypoint being driven to.

The backend decides what a waypoint *is* on the wire (ECEF pose for crl_commander, GeoPose
for nav2), so the conversion is passed in as ``to_src``; this module only knows lat/lon and
metres.
"""

import math
import os

import gpxpy
import numpy as np
import yaml

from robot_mission_planner.follower.road_goal import (
    nearest_index,
    polyline_cumulative,
    remaining_route_length,
)


def resolve_file(name: str, search_dirs) -> str:
    """Absolute path of a route file: as given, or the first hit in ``search_dirs``."""
    if os.path.isabs(name):
        return name
    candidates = [os.path.join(d, name) for d in search_dirs]
    return next(
        (c for c in candidates if os.path.exists(c)),
        candidates[0] if candidates else name,
    )


def load_waypoints(path: str, reverse: bool = False) -> list[dict]:
    """
    Parse a GPX or YAML route file into ``[{lat, lon, ele}, ...]``.

    GPX waypoints first, then track points, then route points: the map_data viewer writes
    waypoints, other tools write tracks. Raises ``ValueError`` for an unknown suffix and lets
    the parsers' own exceptions through.
    """
    points_raw: list[dict] = []
    if path.endswith(".gpx"):
        with open(path, "r") as f:
            gpx = gpxpy.parse(f)
        points = list(gpx.waypoints)
        if not points:
            points = [p for t in gpx.tracks for s in t.segments for p in s.points]
        if not points:
            points = [p for r in gpx.routes for p in r.points]
        for wp in points:
            points_raw.append(
                {"lat": wp.latitude, "lon": wp.longitude, "ele": wp.elevation or 0.0}
            )
    elif path.endswith((".yaml", ".yml")):
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        for wp in data.get("waypoints", []):
            points_raw.append(
                {
                    "lat": wp["latitude"],
                    "lon": wp["longitude"],
                    "ele": wp.get("elevation", 0.0),
                }
            )
    else:
        raise ValueError(
            f"unsupported route file '{path}' (expected .gpx, .yaml or .yml)"
        )
    if reverse:
        points_raw.reverse()
    return points_raw


class Route:
    """
    The waypoints of one leg and the bookkeeping that goes with them.

    ``raw`` is the lat/lon list, ``map_xy`` the same points in ``map_frame`` (``None`` entries
    while the source -> map transform is unknown), ``index`` the waypoint currently driven to.
    ``place()`` (re)computes everything derived; call it whenever the transform changes.
    """

    def __init__(self):
        self.raw: list[dict] = []
        self.source = ""  # "file:<name>" | "plan" | ""
        self.map_xy: list = []
        self.polyline: list = []  # map_xy without the None entries
        self.cum: list = []  # cumulative arclength at every polyline point
        self.seg_a = np.empty((0, 2))  # polyline segments, for distance_to_polyline
        self.seg_b = np.empty((0, 2))
        self.index = 0
        self.synced = False  # the index has been matched to the robot at least once

    def __len__(self) -> int:
        return len(self.raw)

    @property
    def empty(self) -> bool:
        return not self.raw

    @property
    def from_file(self) -> bool:
        return self.source.startswith("file")

    def set(self, points_raw, source: str, index: int = 0) -> None:
        """Replace the waypoints; everything derived is reset (``place()`` fills it in)."""
        self.raw = list(points_raw)
        self.source = source
        self.index = index
        self.synced = False
        self.map_xy = [None] * len(self.raw)
        self._clear_polyline()

    def clear(self) -> None:
        self.set([], "")

    def _clear_polyline(self) -> None:
        self.polyline = []
        self.cum = []
        self.seg_a = self.seg_b = np.empty((0, 2))

    def place(self, frames, to_src) -> None:
        """Put the waypoints in ``map_frame``: ``to_src(point)`` -> (x, y, z), then TF."""
        self.map_xy = [frames.to_map(to_src(p)) for p in self.raw]
        xy = np.array([p for p in self.map_xy if p is not None], dtype=float)
        self.polyline = [(float(p[0]), float(p[1])) for p in xy]
        self.cum = polyline_cumulative(self.polyline)
        if len(xy) >= 2:
            self.seg_a, self.seg_b = xy[:-1], xy[1:]
        else:
            self.seg_a = self.seg_b = np.empty((0, 2))

    # ---------------------------------------------------------------- geometry
    def distance_to(self, idx: int, rob_xy) -> float:
        """Distance (m) from ``rob_xy`` to waypoint ``idx``, or ``inf`` if it has no position."""
        if (
            not (0 <= idx < len(self.map_xy))
            or self.map_xy[idx] is None
            or rob_xy is None
        ):
            return float("inf")
        return math.hypot(
            rob_xy[0] - self.map_xy[idx][0], rob_xy[1] - self.map_xy[idx][1]
        )

    def direction_at(self, idx: int):
        """Unit vector of the route around waypoint ``idx`` in map_frame, or None."""
        if not self.map_xy:
            return None
        n = len(self.map_xy)
        i0, i1 = max(0, min(idx, n - 1)), min(n - 1, idx + 1)
        if i0 == i1:
            i0 = max(0, i1 - 1)
        a, b = self.map_xy[i0], self.map_xy[i1]
        if a is None or b is None:
            return None
        d = np.array([b[0] - a[0], b[1] - a[1]])
        norm = np.linalg.norm(d)
        return d / norm if norm > 1e-6 else None

    def nearest(self, xy) -> int:
        """Index of the waypoint closest to ``xy``."""
        return nearest_index(self.map_xy, xy)

    def remaining_length(self, rob_xy) -> float:
        """Route length (m) still ahead: robot -> waypoint ``index`` -> ... -> last."""
        return remaining_route_length(rob_xy, self.map_xy, self.index)
