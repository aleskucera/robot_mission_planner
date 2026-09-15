import numpy as np
import pytest

from robot_mission_planner.follower.route_shift import centre_points, fit_route_shift

ROBOT = (0.0, 0.0)
STRAIGHT = [(-20.0, 0.0), (-10.0, 0.0), (0.0, 0.0), (10.0, 0.0), (20.0, 0.0)]


def _line(y: float, x0=-2.4, x1=2.4, n=25):
    return np.column_stack([np.linspace(x0, x1, n), np.full(n, y)])


def test_centre_points_crops_to_the_circle_and_the_centre_band():
    xy = np.array([[1.0, 0.0], [1.0, 0.1], [3.0, 0.0]])
    cost = np.array([0.04, 0.8, 0.04])
    assert centre_points(xy, cost, ROBOT, 2.5, 0.086).tolist() == [[1.0, 0.0]]


def test_straight_road_gives_the_lateral_offset_only():
    # Centre cells 1.2 m left of the route, and not symmetric about the robot along it.
    centre = _line(1.2, x0=-1.0, x1=2.4)
    assert fit_route_shift(centre, STRAIGHT, ROBOT, 2.5, 3.0, 15) == pytest.approx(
        (0.0, 1.2), abs=1e-6
    )


def test_bend_inside_the_circle_gives_both_components():
    route = [(-10.0, 0.0), (0.0, 0.0), (0.0, 10.0)]
    shift = np.array([0.5, -0.8])
    centre = np.vstack(
        [
            np.column_stack([np.linspace(-2.0, 0.0, 15), np.zeros(15)]),
            np.column_stack([np.zeros(15), np.linspace(0.1, 2.0, 15)]),
        ]
    ) + shift
    assert fit_route_shift(centre, route, ROBOT, 2.5, 3.0, 15) == pytest.approx(
        tuple(shift), abs=0.05
    )


def test_too_few_centre_points_is_no_measurement():
    assert fit_route_shift(_line(1.0, n=10), STRAIGHT, ROBOT, 2.5, 3.0, 15) is None


def test_route_out_of_reach_is_no_measurement():
    far = [(-20.0, 30.0), (20.0, 30.0)]
    assert fit_route_shift(_line(0.0), far, ROBOT, 2.5, 3.0, 15) is None


def test_shift_is_clamped():
    dx, dy = fit_route_shift(_line(2.5), STRAIGHT, ROBOT, 2.5, 1.0, 15)
    assert np.hypot(dx, dy) == pytest.approx(1.0)
    assert dy > 0
