import math

import pytest

from robot_mission_planner.road_goal import (
    is_behind,
    select_carrot_goal,
    select_path_goal,
    smooth,
)

ROBOT = (0.0, 0.0)
EAST = 0.0


def test_carrot_far_enough_is_returned_as_is():
    g = select_carrot_goal((6.0, 0.0), ROBOT, EAST, min_ahead=4.0, max_ahead=12.0)
    assert g == pytest.approx((6.0, 0.0, 0.0))


def test_carrot_too_close_is_pushed_out_along_its_bearing():
    g = select_carrot_goal((1.0, 1.0), ROBOT, EAST, min_ahead=4.0, max_ahead=12.0)
    assert math.hypot(g[0], g[1]) == pytest.approx(4.0)
    assert g[2] == pytest.approx(math.pi / 4)


def test_carrot_on_the_robot_uses_robot_heading():
    g = select_carrot_goal((0.0, 0.0), ROBOT, math.pi / 2, min_ahead=4.0, max_ahead=12.0)
    assert g == pytest.approx((0.0, 4.0, math.pi / 2))


def test_carrot_beyond_max_is_rejected():
    assert select_carrot_goal((20.0, 0.0), ROBOT, EAST, 4.0, 12.0) is None


def test_path_goal_takes_last_point_within_max():
    path = [(1.0, 0.0), (5.0, 0.0), (9.0, 0.0), (30.0, 0.0)]
    g = select_path_goal(path, ROBOT, EAST, min_ahead=4.0, max_ahead=12.0)
    assert g == pytest.approx((9.0, 0.0, 0.0))


def test_short_path_is_extrapolated_along_its_last_segment():
    # Path bends north-east and ends 2 m from the robot: continue along the bend.
    path = [(0.5, 0.0), (1.0, 0.5), (1.5, 1.0)]
    g = select_path_goal(path, ROBOT, EAST, min_ahead=4.0, max_ahead=12.0)
    assert math.hypot(g[0], g[1]) == pytest.approx(4.0, abs=0.3)
    assert g[2] == pytest.approx(math.pi / 4)
    assert g[0] > 1.5 and g[1] > 1.0


def test_single_close_point_is_pushed_out_along_bearing():
    g = select_path_goal([(0.0, 2.0)], ROBOT, EAST, min_ahead=4.0, max_ahead=12.0)
    assert g == pytest.approx((0.0, 4.0, math.pi / 2))


def test_path_pointing_back_at_robot_falls_back_to_bearing():
    # The last segment heads back towards the robot; walking along it never reaches
    # min_ahead, so the goal is pushed out along the robot -> end bearing instead.
    path = [(3.0, 0.0), (2.0, 0.0), (1.0, 0.0)]
    g = select_path_goal(path, ROBOT, EAST, min_ahead=4.0, max_ahead=12.0)
    assert g == pytest.approx((4.0, 0.0, 0.0))


def test_empty_or_out_of_range_path():
    assert select_path_goal([], ROBOT, EAST, 4.0, 12.0) is None
    assert select_path_goal([(50.0, 0.0)], ROBOT, EAST, 4.0, 12.0) is None


def test_is_behind():
    assert is_behind((-1.0, 0.0), ROBOT, EAST)
    assert not is_behind((1.0, 5.0), ROBOT, EAST)
    assert is_behind((1.0, 0.0), ROBOT, math.pi)


def test_smooth():
    assert smooth(None, (1.0, 1.0), 0.5) == (1.0, 1.0)
    assert smooth((0.0, 0.0), (1.0, 1.0), 0.0) == (1.0, 1.0)
    assert smooth((0.0, 0.0), (1.0, 1.0), 0.5) == pytest.approx((0.5, 0.5))


def test_is_arrived_radius_and_index_guard():
    from robot_mission_planner.road_goal import is_arrived

    wps = [(0.0, 0.0), (10.0, 0.0), (20.0, 0.0), (30.0, 0.0), (40.0, 0.0)]
    assert is_arrived((38.0, 1.0), wps, 4, radius=5.0)
    assert is_arrived((36.0, 0.0), wps, 2, radius=5.0, index_window=3)
    assert not is_arrived((36.0, 0.0), wps, 0, radius=5.0, index_window=3)  # index too early
    assert not is_arrived((30.0, 0.0), wps, 4, radius=5.0)  # 10 m away
    assert not is_arrived((40.0, 0.0), [], 0, radius=5.0)
    assert is_arrived((41.0, 0.0), [None, (40.0, 0.0)], 1, radius=5.0)


# ---------------------------------------------------------------- intersection exit / offsets
from robot_mission_planner.road_goal import nearest_index, passed_along, route_offset_limit  # noqa: E402


def test_nearest_index_skips_missing_points():
    pts = [(0.0, 0.0), None, (10.0, 0.0), (20.0, 0.0)]
    assert nearest_index(pts, (11.0, 1.0)) == 2
    assert nearest_index([None, None], (0.0, 0.0)) == 0


def test_passed_along_right_angle_junction():
    # Route comes from the west, turns north at the node (0, 0). The outgoing direction is
    # north; a robot 3 m north of the node has passed it, 3 m west or east of it has not.
    north = (0.0, 1.0)
    assert passed_along((0.0, 3.0), (0.0, 0.0), north)
    assert not passed_along((-3.0, 0.0), (0.0, 0.0), north)
    assert not passed_along((3.0, 0.0), (0.0, 0.0), north)
    # With the *incoming* direction (east) the same robot north of the node would never pass:
    assert not passed_along((0.0, 3.0), (0.0, 0.0), (1.0, 0.0))


def test_passed_along_without_direction_is_true():
    assert passed_along((0.0, 0.0), (5.0, 5.0), None)


def test_route_offset_limit_relative_to_robot():
    assert route_offset_limit(0.5, 5.0, 2.0, 10.0) == 5.0        # robot on the line: base
    assert route_offset_limit(5.5, 5.0, 2.0, 10.0) == 7.5        # robot off: robot + margin
    assert route_offset_limit(9.5, 5.0, 2.0, 10.0) == 10.0       # capped
    assert route_offset_limit(None, 5.0, 2.0, 10.0) == 5.0       # no pose: base
    assert route_offset_limit(9.5, 5.0, 2.0, 0.0) == 11.5        # no cap
    assert route_offset_limit(30.0, 12.0, 2.0, 10.0) == 12.0     # cap never below the base


# ---------------------------------------------------------------- final approach
from robot_mission_planner.road_goal import remaining_route_length  # noqa: E402


def test_remaining_route_length_counts_robot_and_segments():
    wps = [(0.0, 0.0), (10.0, 0.0), (20.0, 0.0), (30.0, 0.0)]
    # 3 m before waypoint 1, then 10 + 10 m of route left
    assert remaining_route_length((7.0, 0.0), wps, 1) == pytest.approx(23.0)
    assert remaining_route_length((30.0, 0.0), wps, 3) == pytest.approx(0.0)
    # off the line: the robot leg is the straight distance to the current waypoint
    assert remaining_route_length((10.0, 4.0), wps, 2) == pytest.approx(
        math.hypot(10.0, 4.0) + 10.0
    )


def test_remaining_route_length_skips_missing_and_clamps_the_index():
    wps = [(0.0, 0.0), None, (20.0, 0.0)]
    assert remaining_route_length((0.0, 0.0), wps, 0) == pytest.approx(20.0)
    assert remaining_route_length((0.0, 0.0), wps, 99) == pytest.approx(20.0)  # index clamped
    assert remaining_route_length((0.0, 0.0), wps, -5) == pytest.approx(20.0)


def test_remaining_route_length_without_a_usable_route_is_infinite():
    assert remaining_route_length((0.0, 0.0), [], 0) == float("inf")
    assert remaining_route_length((0.0, 0.0), [None, None], 0) == float("inf")


# ---------------------------------------------------------------- QR goal distance
from robot_mission_planner.road_goal import latlon_distance  # noqa: E402


def test_latlon_distance_metres():
    assert latlon_distance((50.11, 14.41), (50.11, 14.41)) == pytest.approx(0.0)
    # 0.001 deg of latitude is ~111.3 m anywhere
    assert latlon_distance((50.11, 14.41), (50.111, 14.41)) == pytest.approx(111.32, abs=0.5)
    # the same step in longitude is shorter by cos(lat) at 50 deg
    assert latlon_distance((50.11, 14.41), (50.11, 14.411)) == pytest.approx(
        111.32 * math.cos(math.radians(50.11)), abs=0.5
    )
    # the start code seen again: well inside the 2 m pending-goal threshold
    assert latlon_distance((50.1103476, 14.4159857), (50.1103480, 14.4159860)) < 2.0
