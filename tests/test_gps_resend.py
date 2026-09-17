"""A GPS goal the backend dropped is re-sent by the tick in every mode (nav2 in road_gps)."""

from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
from robot_mission_planner.follower.modes import ROAD_GPS  # noqa: E402
from robot_mission_planner.road_follower import RoadFollower  # noqa: E402


def _node(sent: list, pending_timer=None):
    return SimpleNamespace(
        mode=ROAD_GPS,
        state=RoadFollower.STATE_GPS,
        STATE_ROAD=RoadFollower.STATE_ROAD,
        STATE_GPS=RoadFollower.STATE_GPS,
        STATE_IDLE=RoadFollower.STATE_IDLE,
        STATE_PLANNING=RoadFollower.STATE_PLANNING,
        STATE_ARRIVED=RoadFollower.STATE_ARRIVED,
        loop=False,
        waypoints_map=[(0.0, 0.0), (50.0, 0.0), (100.0, 0.0)],
        current_waypoint_index=1,
        goal_reached_radius=5.0,
        arrival_index_window=3,
        road_goal_source="carrot",
        _goal_active=False,  # nav2 reported the sequence aborted
        _pending_goal_timer=pending_timer,
        backend=SimpleNamespace(
            reports_progress=True,
            take_restarted=lambda: False,
            left_us=lambda active: False,
        ),
        _publish_state=lambda: None,
        _sync_qr_detection=lambda: None,
        _robot_pose=lambda: (50.0, 0.0, 0.0),
        _enter_arrived=lambda xy: pytest.fail("not at the goal"),
        _send_gps_goal=lambda: sent.append(1),
        _check_state_transitions=lambda xy: None,
    )


def test_dropped_gps_goal_is_resent_in_road_gps():
    sent = []
    RoadFollower._main_logic_step(_node(sent))
    assert sent == [1]


def test_not_while_a_hand_over_pause_runs():
    sent = []
    RoadFollower._main_logic_step(_node(sent, pending_timer=object()))
    assert sent == []
