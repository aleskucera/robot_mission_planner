"""A file route ends like a mission route: ARRIVED at its last waypoint, unless it loops."""

from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
from robot_mission_planner.follower.modes import GPS  # noqa: E402
from robot_mission_planner.road_follower import RoadFollower  # noqa: E402


def _node(loop: bool, arrived: list):
    return SimpleNamespace(
        mode=GPS,
        state=RoadFollower.STATE_GPS,
        STATE_ROAD=RoadFollower.STATE_ROAD,
        STATE_GPS=RoadFollower.STATE_GPS,
        STATE_IDLE=RoadFollower.STATE_IDLE,
        STATE_PLANNING=RoadFollower.STATE_PLANNING,
        STATE_ARRIVED=RoadFollower.STATE_ARRIVED,
        loop=loop,
        _mission_goal=None,  # a file route
        waypoints_map=[(0.0, 0.0), (5.0, 0.0), (10.0, 0.0)],
        current_waypoint_index=2,
        goal_reached_radius=5.0,
        arrival_index_window=3,
        road_goal_source="carrot",
        _goal_active=True,
        _pending_goal_timer=None,
        backend=SimpleNamespace(
            reports_progress=True,
            take_restarted=lambda: False,
            left_us=lambda active: False,
        ),
        _publish_state=lambda: None,
        _sync_qr_detection=lambda: None,
        _robot_pose=lambda: (10.0, 1.0, 0.0),
        _enter_arrived=lambda xy: arrived.append(xy),
        _send_gps_goal=lambda: None,
        _check_state_transitions=lambda xy: None,
    )


def test_file_route_arrives_at_its_last_waypoint():
    arrived = []
    RoadFollower._main_logic_step(_node(loop=False, arrived=arrived))
    assert arrived == [(10.0, 1.0)]


def test_looping_file_route_keeps_driving():
    arrived = []
    RoadFollower._main_logic_step(_node(loop=True, arrived=arrived))
    assert arrived == []
