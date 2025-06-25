import numpy as np
from shapely.geometry import MultiPoint


def get_point_line(p1, p2, density, increase=0):
    """
    Get the line between two points.

    Parameters:
    -----------
    p1 : shapely.geometry.Point
        First point.
    p2 : shapely.geometry.Point
        Second point.
    density : float
        Density of the line.
    increase : int
        Increase the line by this many points.

    Returns:
    --------
    vec : np.array
        Vector between the two points.
    line : np.array
        Line between the two points.
    dist_line : np.array
        Distance of each point in the line from the first point.
    """
    x1 = p1.x
    y1 = p1.y
    x2 = p2.x
    y2 = p2.y

    a = np.array([x1, y1])
    b = np.array([x2, y2])

    vec = (b - a).T

    # Ceil, because more points is OK, while less points could be problematic (distance between points should not be larger than 1*density)
    line = get_equidistant_points(a, b, int(np.ceil(np.linalg.norm(vec) / density)) + 1)
    dist_line = np.zeros((len(line), 1))

    if increase > 0:
        line, dist_line = increase_line(line, dist_line, b - a, increase, density)

    return vec, line, dist_line


def increase_line(line, dist_line, vec, n, density):
    """
    Increase the line by n points in the direction of vec.

    Parameters:
    -----------
    line : np.array
        Line to increase.
    dist_line : np.array
        Distance of each point in the line from the first point.
    vec : np.array
        Vector to increase the line in.
    n : int
        Number of points to increase the line by.
    density : float
        Density of the line.

    Returns:
    --------
    line : np.array
        Increased line.
    dist_line : np.array
        Distance of each point in the increased line from the first point.
    """
    vec = vec / np.linalg.norm(vec)
    arange_increase_vec = density * np.arange(1, n + 1)
    arange_increase_vec = arange_increase_vec.reshape((-1, 1))
    increase_vec = arange_increase_vec * vec
    before = line[0] * np.ones((n, 2)) - increase_vec
    before = np.flip(before, axis=0)
    after = line[-1] * np.ones((n, 2)) + increase_vec

    line = np.concatenate((before, line, after), axis=0)
    dist_line = np.concatenate(
        (np.flip(arange_increase_vec, axis=0), dist_line, arange_increase_vec), axis=0
    )
    return line, dist_line


def get_equidistant_points(p1, p2, n):
    """
    Split the line between p1 and p2 into n equidistant points.

    Parameters:
    -----------
    p1 : np.array
        First point.
    p2 : np.array
        Second point.
    n : int
        Number of points.

    Returns:
    --------
    np.array
        Equidistantly spaced points.
    """
    return np.concatenate(
        (
            np.expand_dims(np.linspace(p1[0], p2[0], max(n, 2)), axis=1),
            np.expand_dims(np.linspace(p1[1], p2[1], max(n, 2)), axis=1),
        ),
        axis=1,
    )


def points_to_graph_points(point1, point2, density=1, width=10):
    """
    Transform point into graph point.

    Parameters:
    -----------
    points1 : shapely.geometry.Point
        First point.
    points2 : shapely.geometry.Point
        Second point.
    density : float
        Density of the line.
    width : float
        Width of the line.

    Returns:
    --------
    all_points : shapely.geometry.MultiPoint
        All points.
    point_line : shapely.geometry.MultiPoint
        Points in the line.
    dist_from_line : np.array
        Distance of each point from the line.
    """
    perpendicular_increase = int(round(width / 2 / density))
    parallel_increase = int(round(width / 4 / density))

    if point1.bounds == point2.bounds:
        return (
            MultiPoint([[point1.x, point1.y]]),
            MultiPoint([[point1.x, point1.y]]),
            np.zeros((1, 1)),
        )
    else:
        vec, point_line, dist_line = get_point_line(
            point1, point2, density, parallel_increase
        )

    normal_vec = np.matmul(
        np.array(
            [
                [np.cos(np.pi / 2), -np.sin(np.pi / 2)],
                [np.sin(np.pi / 2), np.cos(np.pi / 2)],
            ]
        ),
        vec,
    ) / np.linalg.norm(vec)

    points_in_line = point_line.shape[0]

    all_points = np.zeros((points_in_line * (perpendicular_increase * 2 + 1), 2))
    dist_from_line = np.zeros((points_in_line * (perpendicular_increase * 2 + 1), 1))

    line_start_index = points_in_line * perpendicular_increase
    line_end_index = points_in_line * (perpendicular_increase + 1) - 1

    all_points[line_start_index : line_end_index + 1, :] = point_line
    dist_from_line[line_start_index : line_end_index + 1] = dist_line

    for i in range(perpendicular_increase):
        pos_line_points = point_line + normal_vec * density * (i + 1)
        neg_line_points = point_line - normal_vec * density * (i + 1)
        all_points[
            points_in_line * (perpendicular_increase + i + 1) : points_in_line
            * (perpendicular_increase + i + 2),
            :,
        ] = pos_line_points
        all_points[
            points_in_line * (perpendicular_increase - i - 1) : points_in_line
            * (perpendicular_increase - i),
            :,
        ] = neg_line_points
        dist_from_line[
            points_in_line * (perpendicular_increase + i + 1) : points_in_line
            * (perpendicular_increase + i + 2)
        ] = dist_line + (i + 1) * density
        dist_from_line[
            points_in_line * (perpendicular_increase - i - 1) : points_in_line
            * (perpendicular_increase - i)
        ] = dist_line + (i + 1) * density

    all_points = MultiPoint(all_points)
    point_line = MultiPoint(point_line)

    return all_points, point_line, dist_from_line
