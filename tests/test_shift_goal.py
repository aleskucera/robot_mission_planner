"""gps_shift: the next goto goes out once the *shifted* waypoint is reached, no commander STOP."""

from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
from robot_mission_planner.follower.modes import GPS_SHIFT  # noqa: E402
from robot_mission_planner.follower.route import Route  # noqa: E402
from robot_mission_planner.road_follower import RoadFollower  # noqa: E402


def test_next_goal_is_sent_at_the_shifted_waypoint():
    route = Route()
    route.set([{}] * 4, "test")
    route.map_xy = [(0.0, 0.0), (5.0, 0.0), (10.0, 0.0), (15.0, 0.0)]
    sent = []
    node = SimpleNamespace(
        mode=GPS_SHIFT,
        route=route,
        waypoints_map=route.map_xy,
        current_waypoint_index=1,
        pose_gps=None,
        road_goal_min_ahead=4.0,
        road_goal_reached_distance=2.5,
        road_goal_update_distance=1.0,
        _shift=(0.0, 3.0),  # road 3 m left of the route
        _goal_active=True,
        _last_road_goal=(5.0, 3.0),
        backend=SimpleNamespace(
            left_us=lambda active: False, send_pose=lambda x, y, yaw: sent.append((x, y))
        ),
        get_logger=lambda: SimpleNamespace(info=lambda *a, **k: None),
        map_frame="map",
    )
    node._waypoint_distance = lambda i, xy: RoadFollower._waypoint_distance(node, i, xy)
    # 3 m before the shifted waypoint 1 but 4.2 m from the raw one: the goal must move on.
    node._robot_pose = lambda: (2.0, 3.0, 0.0)
    node._road_goal_needs_update = lambda g: RoadFollower._road_goal_needs_update(node, g)

    RoadFollower._send_shift_goal(node)

    assert sent == [(10.0, 3.0)]
