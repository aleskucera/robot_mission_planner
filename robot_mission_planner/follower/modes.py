"""What the follower is doing: the three ways of getting the robot somewhere.

Everything below the mode is shared -- the frames, the route, the road-goal geometry, the
backend, the mission layer. A mode only says *which* of those take part:

``road_gps``  follow the visually detected road, and hand over to the route's waypoints
              around OSM intersections, when the road detection drops out, when the
              commander reports being stuck, and for the last metres to the goal.
``gps``       follow the route's waypoints from beginning to end and never look at the road.
``road``      follow the road and nothing else: no route, no intersections, no goal.

The route of the two route modes comes either from a file (``file:`` a GPX or YAML) or from
a mission goal (a QR code -> ``PlanRoute``); that is a separate choice, not a mode.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Mode:
    name: str
    road: bool  # road observations produce goals
    route: bool  # a route of waypoints is loaded and followed
    switching: bool  # ROAD <-> GPS hand-over during the run
    description: str

    @property
    def mission(self) -> bool:
        """Whether a QR/point goal can be planned into a route for this mode."""
        return self.route


ROAD_GPS = Mode(
    "road_gps",
    road=True,
    route=True,
    switching=True,
    description="road following, GPS waypoints at intersections and for the final approach",
)
GPS = Mode(
    "gps",
    road=False,
    route=True,
    switching=False,
    description="the route's waypoints only, no road following",
)
ROAD = Mode(
    "road",
    road=True,
    route=False,
    switching=False,
    description="road following only, no route and no goal",
)

MODES = {m.name: m for m in (ROAD_GPS, GPS, ROAD)}
NAMES = tuple(MODES)


def get(name: str) -> Mode:
    """The mode called ``name``; raises ``KeyError`` with the valid names in the message."""
    try:
        return MODES[name]
    except KeyError:
        raise KeyError(f"unknown mode '{name}', expected one of {NAMES}") from None
