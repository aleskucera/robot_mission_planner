"""Mission planning: a timed-out PlanRoute attempt is cancelled and its late answers ignored."""

from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
from robot_mission_planner.road_follower import RoadFollower  # noqa: E402


class _Future:
    def __init__(self, value):
        self._value = value

    def result(self):
        return self._value


def _node(attempt: int):
    return SimpleNamespace(
        state=RoadFollower.STATE_PLANNING,
        STATE_PLANNING=RoadFollower.STATE_PLANNING,
        _plan_attempt=attempt,
        _plan_goal_handle=None,
        _plan_stale=lambda a: RoadFollower._plan_stale(node, a),
        _plan_failed=lambda *a, **k: failed.append(a),
        get_logger=lambda: SimpleNamespace(warning=lambda *a, **k: None),
    )


def test_late_answer_of_a_timed_out_attempt_is_cancelled_and_ignored():
    global node, failed
    failed = []
    node = _node(attempt=2)  # attempt 1 timed out, attempt 2 is in flight
    cancelled = []
    handle = SimpleNamespace(accepted=True, cancel_goal_async=lambda: cancelled.append(1))

    RoadFollower._plan_response_cb(node, _Future(handle), attempt=1)

    assert cancelled == [1]
    assert node._plan_goal_handle is None
    assert failed == []


def test_current_attempt_is_kept():
    global node, failed
    failed = []
    node = _node(attempt=2)
    results = []
    handle = SimpleNamespace(
        accepted=True,
        cancel_goal_async=lambda: pytest.fail("cancelled the live attempt"),
        get_result_async=lambda: SimpleNamespace(add_done_callback=results.append),
    )

    RoadFollower._plan_response_cb(node, _Future(handle), attempt=2)

    assert node._plan_goal_handle is handle
    assert len(results) == 1


def test_plan_failed_cancels_the_goal_in_flight():
    cancelled = []
    node = SimpleNamespace(
        _plan_goal_handle=SimpleNamespace(cancel_goal_async=lambda: cancelled.append(1)),
        get_logger=lambda: SimpleNamespace(warning=lambda *a, **k: None),
    )
    RoadFollower._cancel_plan_goal(node)
    assert cancelled == [1] and node._plan_goal_handle is None
