"""Coordinate frames and transforms for the follower.

Everything the follower compares is in ``map_frame`` (``FP_ENU0`` on Helhest). Route
waypoints arrive as lat/lon, are converted to the backend's source frame (ECEF for
crl_commander, UTM for nav2) and are placed in ``map_frame`` through TF; road observations
arrive in whatever frame their sensor uses and are transformed the same way.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from ros2_numpy import numpify
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

# WGS84 (lat/lon -> ECEF for the commander backend; kept local to avoid a map_data dependency)
_WGS84_A = 6378137.0
_WGS84_E2 = (1.0 / 298.257223563) * (2.0 - 1.0 / 298.257223563)

# How far the waypoint source frame -> map_frame transform has to move before the waypoints
# are re-placed (F9): below 0.1 m, or ~0.04 deg of rotation (Frobenius norm of the matrix
# difference), it is TF noise rather than a new ENU origin.
TF_SHIFT_EPS = 0.1
TF_ROTATION_EPS = 1e-3


def latlon_to_ecef(
    lat_deg: float, lon_deg: float, alt_m: float = 0.0
) -> tuple[float, float, float]:
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * math.sin(lat) ** 2)
    x = (n + alt_m) * math.cos(lat) * math.cos(lon)
    y = (n + alt_m) * math.cos(lat) * math.sin(lon)
    z = (n * (1.0 - _WGS84_E2) + alt_m) * math.sin(lat)
    return x, y, z


def transform_xyz(
    matrix: np.ndarray, x: float, y: float, z: float = 0.0
) -> tuple[float, float, float]:
    """Apply a 4x4 homogeneous matrix to a point."""
    p = matrix[:3, :3] @ np.array([x, y, z]) + matrix[:3, 3]
    return float(p[0]), float(p[1]), float(p[2])


def marker_point_in_header_frame(marker: Marker, point):
    """A Marker ``points[]`` entry (relative to ``marker.pose``) in the marker's own frame."""
    m = numpify(marker.pose)
    x, y, z = transform_xyz(m, point.x, point.y, point.z)
    return Point(x=float(x), y=float(y), z=float(z))


def distance_to_polyline(
    point, segments_a: np.ndarray, segments_b: np.ndarray
) -> float:
    """Minimum distance from ``point`` (x, y) to the polyline given as segment endpoints."""
    if segments_a.size == 0:
        return float("inf")
    p = np.asarray(point, dtype=float)
    ab = segments_b - segments_a
    ap = p - segments_a
    denom = np.einsum("ij,ij->i", ab, ab)
    t = np.where(
        denom > 0, np.einsum("ij,ij->i", ap, ab) / np.where(denom > 0, denom, 1.0), 0.0
    )
    t = np.clip(t, 0.0, 1.0)
    closest = segments_a + ab * t[:, None]
    return float(np.min(np.hypot(*(p - closest).T)))


class Frames:
    """The follower's TF buffer and the frames it works in."""

    def __init__(self, node, *, map_frame: str, robot_frame: str, source_frame: str):
        self.node = node
        self.map_frame = map_frame
        self.robot_frame = robot_frame
        self.source_frame = source_frame  # the frame route waypoints are expressed in
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.src_to_map = None  # 4x4, source_frame -> map_frame

    @property
    def ready(self) -> bool:
        """True once the route waypoints can be placed in ``map_frame``."""
        return self.src_to_map is not None

    def matrix(self, target: str, source: str, timeout: float = 0.5):
        """4x4 matrix mapping points in ``source`` into ``target``, or None."""
        try:
            tf_msg = self.buffer.lookup_transform(
                target,
                source,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=timeout),
            )
        except Exception as e:  # TransformException and friends
            self.node.get_logger().warning(
                f"TF {target} <- {source} unavailable: {e}", throttle_duration_sec=5.0
            )
            return None
        return numpify(tf_msg.transform)

    def robot_pose(self):
        """Robot (x, y, yaw) in map_frame, or None."""
        m = self.matrix(self.map_frame, self.robot_frame, timeout=0.2)
        if m is None:
            return None
        return float(m[0, 3]), float(m[1, 3]), float(math.atan2(m[1, 0], m[0, 0]))

    def pose_to_map(self, pose: PoseStamped):
        """(x, y) of a PoseStamped in map_frame, or None."""
        if not pose.header.frame_id or pose.header.frame_id == self.map_frame:
            return pose.pose.position.x, pose.pose.position.y
        m = self.matrix(self.map_frame, pose.header.frame_id, timeout=0.2)
        if m is None:
            return None
        x, y, _ = transform_xyz(
            m, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
        )
        return x, y

    def to_map(self, xyz) -> tuple[float, float] | None:
        """A point in ``source_frame`` placed in ``map_frame`` (None before the transform is known)."""
        if self.src_to_map is None:
            return None
        x, y, _ = transform_xyz(self.src_to_map, *xyz)
        return x, y

    def refresh_source(self, timeout: float = 1.0) -> str | None:
        """
        Look the source frame -> map_frame transform up again.

        ``"first"`` when it was resolved for the first time, ``"moved"`` when it has changed
        since (a Fixposition restart re-defines FP_ENU0, whose origin is the run's first fix,
        and every waypoint would otherwise stay where the old origin put it), ``None`` when
        there is nothing to do. The caller re-places whatever it derived from the transform.
        """
        m = self.matrix(self.map_frame, self.source_frame, timeout=timeout)
        if m is None:
            return None
        if self.src_to_map is None:
            self.src_to_map = m
            return "first"
        shift = float(np.linalg.norm(m[:3, 3] - self.src_to_map[:3, 3]))
        rotation = float(np.linalg.norm(m[:3, :3] - self.src_to_map[:3, :3]))
        if shift <= TF_SHIFT_EPS and rotation <= TF_ROTATION_EPS:
            return None
        self.node.get_logger().warning(
            f"{self.source_frame} -> {self.map_frame} moved by {shift:.2f} m "
            f"(rotation {rotation:.4f}): re-placing the waypoints. "
            f"{self.map_frame} was most likely redefined by a Fixposition restart."
        )
        self.src_to_map = m
        return "moved"
