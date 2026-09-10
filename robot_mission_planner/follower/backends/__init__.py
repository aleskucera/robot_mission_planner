"""Navigation backends: what the follower talks to once it has decided where to drive.

``commander`` is the Helhest field stack (crl_commander on the NUC), ``nav2`` the Nav2
action servers, ``follow_path`` a bare nav2 ``FollowPath`` controller (``path_follower``)
with no global planner. They differ in what a goal *is* -- a pose, a waypoint sequence or a
path -- and in the frame waypoints go out in; everything above them works in ``map_frame``.
"""

from robot_mission_planner.follower.backends.base import Backend
from robot_mission_planner.follower.backends.commander import CommanderBackend
from robot_mission_planner.follower.backends.follow_path import FollowPathBackend
from robot_mission_planner.follower.backends.nav2 import Nav2Backend

KINDS = ("commander", "nav2", "follow_path")


def make_backend(kind: str, node, frames, **kwargs) -> Backend:
    """Build the backend named ``kind`` (see ``KINDS``)."""
    if kind == "commander":
        return CommanderBackend(node, frames, **kwargs)
    if kind == "nav2":
        return Nav2Backend(node, frames, **kwargs)
    if kind == "follow_path":
        return FollowPathBackend(node, frames, **kwargs)
    raise ValueError(f"unknown nav_backend '{kind}', expected one of {KINDS}")


__all__ = ["Backend", "CommanderBackend", "FollowPathBackend", "Nav2Backend", "KINDS", "make_backend"]
