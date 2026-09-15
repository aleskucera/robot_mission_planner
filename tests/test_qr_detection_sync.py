"""road_follower pauses qr_goal detection while a leg runs: only changes reach the service."""

from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
from robot_mission_planner.road_follower import RoadFollower  # noqa: E402


class _Client:
    def __init__(self):
        self.ready, self.sent = False, []

    def service_is_ready(self):
        return self.ready

    def call_async(self, req):
        self.sent.append(req.data)


def test_sync_sends_only_changes_once_the_service_is_up():
    client = _Client()
    node = SimpleNamespace(
        _qr_detection_client=client,
        _qr_detection_wanted=False,
        _qr_detection_sent=None,
        get_logger=lambda: SimpleNamespace(info=lambda *_: None),
    )
    sync = lambda: RoadFollower._sync_qr_detection(node)  # noqa: E731

    sync()
    assert client.sent == []  # qr_goal not up yet: retried next tick
    client.ready = True
    sync()
    sync()
    assert client.sent == [False]
    node._qr_detection_wanted = True  # arrived / aborted
    sync()
    assert client.sent == [False, True]
