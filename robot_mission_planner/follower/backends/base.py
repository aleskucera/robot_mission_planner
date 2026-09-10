"""What the follower needs from a navigation stack."""

from robot_mission_planner.follower.frames import latlon_to_ecef  # noqa: F401  (re-export for backends)


class Backend:
    """
    One goal at a time, in ``map_frame``, plus the waypoint conversion the stack expects.

    The follower owns the decision (*where* to drive and *when* to hand over between road
    and GPS); a backend owns the wire format and the bookkeeping of the stack it talks to.
    It reports back through the two hooks, which the follower replaces at construction:

    ``on_goal_inactive()``      there is no goal being driven any more (finished, aborted,
                                rejected, or close enough to ask for the next one)
    ``on_waypoint_reached(i)``  the stack says it is now driving to waypoint ``i``
    """

    kind = "base"
    geo_goals = False  # True: waypoints go out as lat/lon and need no map_frame placement
    supports_sequence = True  # False: the backend can only be given one pose at a time

    def __init__(self, node, frames):
        self.node = node
        self.frames = frames
        self.log = node.get_logger()
        self.on_goal_inactive = lambda: None
        self.on_waypoint_reached = lambda index: None

    # ---------------------------------------------------------------- waypoints
    def to_src(self, point) -> tuple[float, float, float]:
        """``{lat, lon, ele}`` -> (x, y, z) in the frame this backend sends waypoints in."""
        raise NotImplementedError

    def waypoint_msg(self, point):
        """``{lat, lon, ele}`` -> the message this backend wants one waypoint as."""
        raise NotImplementedError

    # ---------------------------------------------------------------- goals
    def send_pose(self, x: float, y: float, yaw: float) -> None:
        """Drive to one pose in ``map_frame``."""
        raise NotImplementedError

    def send_sequence(self, waypoints, loop: bool = False) -> None:
        """Drive through ``waypoints`` (as returned by :meth:`waypoint_msg`), in order."""
        raise NotImplementedError

    def cancel(self) -> None:
        """Give up the current goal and stand still."""
        raise NotImplementedError

    def hand_over(self, to_gps: bool) -> None:
        """
        Called before the goal of a new follower state is sent.

        The commander switches goto <-> sequence by itself, so nothing has to stop first;
        backends that cannot do that cancel here (see ``stop_between_modes``).
        """

    # ---------------------------------------------------------------- status
    @property
    def state_text(self) -> str | None:
        """What the stack says it is doing, for the operator's status line."""
        return None

    def stuck(self) -> bool:
        return False

    def take_restarted(self) -> bool:
        """True once after the stack was seen to restart (the caller re-sends its goal)."""
        return False

    def left_us(self, goal_active: bool) -> bool:
        """True when the stack stopped driving a goal we still believe in."""
        return False
