import math

import pytest

from robot_mission_planner.road_goal import (
    is_behind,
    polyline_cumulative,
    project_on_route,
    route_point_at,
    select_carrot_goal,
    select_path_goal,
    select_route_goal,
    smooth,
    turn_limited_arclength,
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


# ---------------------------------------------------------------- ring pruning (P4)
from robot_mission_planner.road_goal import indices_near_polyline  # noqa: E402


def test_indices_near_polyline_keeps_only_the_rings_on_the_route():
    # Route east along y = 0, then north at (20, 0).
    route = [(0.0, 0.0), (10.0, 0.0), (20.0, 0.0), (20.0, 10.0)]
    rings = [
        (5.0, 1.0),    # on the route
        (12.0, 8.0),   # a side junction 8 m off it
        (20.0, 0.0),   # the corner node itself
        (23.5, 5.0),   # 3.5 m off the northbound leg
        (20.0, 12.0),  # 2 m past the end of the route
    ]
    assert indices_near_polyline(rings, route, 3.0) == [0, 2, 4]


def test_indices_near_polyline_off_and_without_a_route():
    rings = [(5.0, 1.0), (12.0, 8.0)]
    route = [(0.0, 0.0), (10.0, 0.0)]
    assert indices_near_polyline(rings, route, 0.0) == [0, 1]  # 0 = filter off
    assert indices_near_polyline(rings, [(0.0, 0.0)], 3.0) == [0, 1]  # no polyline yet
    assert indices_near_polyline([], route, 3.0) == []


def test_indices_near_polyline_measures_to_the_segment_not_the_vertices():
    # Midway between two waypoints 20 m apart: 1 m from the segment, 10 m from either end.
    route = [(0.0, 0.0), (20.0, 0.0)]
    assert indices_near_polyline([(10.0, 1.0)], route, 3.0) == [0]
    assert indices_near_polyline([(10.0, 5.0)], route, 3.0) == []
    # Before the start of the route the distance is the one to the first vertex.
    assert indices_near_polyline([(-4.0, 0.0)], route, 3.0) == []


# --- road_goal_source: route -------------------------------------------------------------
# A straight route east, waypoints every 3 m as route_planner resamples them, and a route
# that turns 90 degrees north after 12 m.
STRAIGHT = [(float(x), 0.0) for x in range(0, 31, 3)]
STRAIGHT_CUM = polyline_cumulative(STRAIGHT)
CORNER = [(0.0, 0.0), (6.0, 0.0), (12.0, 0.0), (12.0, 6.0), (12.0, 12.0)]
CORNER_CUM = polyline_cumulative(CORNER)


def route_goal(carrot, robot, points=STRAIGHT, cum=STRAIGHT_CUM, index=0, **kw):
    kw.setdefault("stretch", 6.0)
    kw.setdefault("min_ahead", 4.0)
    kw.setdefault("max_ahead", 12.0)
    kw.setdefault("lateral_limit", 5.0)
    return select_route_goal(carrot, robot, points, cum, index, **kw)


def test_project_on_route_gives_arclength_and_signed_offset():
    s, n = project_on_route(STRAIGHT, STRAIGHT_CUM, (7.0, 2.0))
    assert (s, n) == pytest.approx((7.0, 2.0))          # 2 m to the left of an eastward route
    s, n = project_on_route(STRAIGHT, STRAIGHT_CUM, (7.0, -2.0))
    assert (s, n) == pytest.approx((7.0, -2.0))


def test_project_on_route_window_keeps_the_current_leg():
    # A route that folds back 4 m north: without a window the point projects onto the
    # return leg, with one it stays on the leg around the current waypoint index.
    pts = [(0.0, 0.0), (10.0, 0.0), (20.0, 0.0), (20.0, 4.0), (10.0, 4.0), (0.0, 4.0)]
    cum = polyline_cumulative(pts)
    assert project_on_route(pts, cum, (10.0, 3.0))[0] == pytest.approx(cum[4])
    assert project_on_route(pts, cum, (10.0, 3.0), index=1, window=1)[0] == pytest.approx(10.0)


def test_route_point_at_extrapolates_past_the_ends():
    (x, y), yaw = route_point_at(STRAIGHT, STRAIGHT_CUM, 34.0)
    assert (x, y, yaw) == pytest.approx((34.0, 0.0, 0.0))


def test_lagging_carrot_still_gives_a_goal_stretched_along_the_route():
    # The hull centre sits 1 m behind the robot: the plain carrot source would send a goal
    # 4 m along the robot heading, the route source puts it 6 m further along the route.
    g = route_goal((9.0, 0.0), (10.0, 0.0))
    assert g == pytest.approx((16.0, 0.0, 0.0))


def test_carrot_offset_is_carried_to_the_stretched_goal():
    # The real path runs 2 m north of the OSM line here: the goal keeps that offset.
    g = route_goal((10.0, 2.0), (10.0, 2.0))
    assert g == pytest.approx((16.0, 2.0, 0.0))


def test_goal_follows_the_corner_instead_of_the_robot_heading():
    g = route_goal((9.0, 0.0), (9.0, 0.0), points=CORNER, cum=CORNER_CUM, index=1, max_turn=0.0)
    assert g == pytest.approx((12.0, 3.0, math.pi / 2))


def test_corner_clamp_stops_the_stretch_at_a_sharp_turn():
    g = route_goal(
        (9.0, 0.0), (5.0, 0.0), points=CORNER, cum=CORNER_CUM, index=1,
        max_turn=math.radians(45), min_ahead=4.0,
    )
    assert g == pytest.approx((12.0, 0.0, 0.0))          # stopped at the corner vertex


def test_min_ahead_wins_over_the_corner_clamp():
    # Standing on the corner: clamping would give a goal inside the commander's arrival box,
    # so the goal is pushed on around the corner instead.
    g = route_goal(
        (11.0, 0.0), (11.0, 0.0), points=CORNER, cum=CORNER_CUM, index=2,
        max_turn=math.radians(45),
    )
    assert math.hypot(g[0] - 11.0, g[1]) >= 4.0 - 1e-6
    assert g[1] > 0.0


def test_stretch_is_shortened_to_max_ahead():
    g = route_goal((10.0, 0.0), (10.0, 0.0), stretch=20.0, max_ahead=8.0)
    assert g == pytest.approx((18.0, 0.0, 0.0))


def test_lateral_offset_is_clamped_and_scaled():
    g = route_goal((10.0, 9.0), (10.0, 9.0), lateral_limit=3.0)
    assert g == pytest.approx((16.0, 3.0, 0.0))
    g = route_goal((10.0, 4.0), (10.0, 4.0), lateral_gain=0.5)
    assert g == pytest.approx((16.0, 2.0, 0.0))


def test_carrot_ahead_of_the_robot_moves_the_goal_further():
    g = route_goal((14.0, 0.0), (10.0, 0.0))
    assert g == pytest.approx((20.0, 0.0, 0.0))


def test_without_a_carrot_the_robot_offset_is_kept():
    g = route_goal(None, (10.0, 1.5))
    assert g == pytest.approx((16.0, 1.5, 0.0))


def test_no_goal_without_a_usable_route():
    assert route_goal((1.0, 0.0), (0.0, 0.0), points=[(0.0, 0.0)], cum=[0.0]) is None


def test_the_offset_is_kept_however_far_the_route_is():
    # The whole point of using the route relatively: a robot 20 m off the mapped line (or a
    # 20 m GNSS error) still gets a goal 6 m ahead of itself, not one pulling it to the line.
    assert route_goal((0.0, 20.0), (0.0, 20.0), lateral_limit=0.0) == pytest.approx(
        (6.0, 20.0, 0.0)
    )


def test_no_goal_when_the_clamped_route_is_out_of_reach():
    # 20 m off the route but the offset may only be 3 m: the goal would be 17 m away, so the
    # projection is not trusted at all (in the follower the carrot is rejected first).
    assert route_goal((0.0, 20.0), (0.0, 20.0), lateral_limit=3.0) is None


def test_turn_limited_arclength_without_a_limit():
    assert turn_limited_arclength(CORNER, CORNER_CUM, 0.0, 18.0, 0.0) == 18.0
