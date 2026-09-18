"""An aborted leg leaves no route behind: rviz, the HUD and the viewer must drop it."""

from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
from robot_mission_planner.follower.route import Route  # noqa: E402
from robot_mission_planner.road_follower import RoadFollower  # noqa: E402


def _node(published: dict):
    route = Route()
    route.set([{"lat": 50.0, "lon": 14.0}, {"lat": 50.001, "lon": 14.001}], "route_planner")
    return SimpleNamespace(
        route=route,
        _pending_goal=(50.0, 14.0),
        get_logger=lambda: SimpleNamespace(warning=lambda *_: None, info=lambda *_: None),
        _state_text=lambda: "GPS:planned",
        _cancel_plan_timer=lambda: None,
        _cancel_pending_goal_timer=lambda: None,
        _cancel_current_goal=lambda: None,
        _cancel_plan_goal=lambda: None,
        _publish_route=lambda: published.__setitem__("route", len(route)),
        _publish_waypoints_markers=lambda: published.__setitem__("markers", True),
        _event=lambda text: published.__setitem__("event", text),
        _enter_idle=lambda why: published.__setitem__("idle", why),
    )


def test_abort_clears_the_route_and_republishes_it():
    published = {}
    node = _node(published)
    response = SimpleNamespace(success=False, message="")

    RoadFollower._abort_callback(node, SimpleNamespace(), response)

    assert node.route.empty  # nothing is being followed any more
    assert published["route"] == 0  # the empty Path went out, so consumers drop the route
    assert published["markers"] is True
    assert published["event"].startswith("ABORT:")
    assert published["idle"] == "aborted by operator"
    assert response.success
