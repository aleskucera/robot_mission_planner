import os
import pickle

import numpy as np
from shapely.geometry import Point

from map_data import MapData, CoordsData

def replan_path(path, grid):
    pass

def create_grid(low, high, cell_size=0.25):
    '''
    Create a grid of points.

    Parameters:
    -----------
    low : tuple
        Lower bounds of the grid.
    high : tuple
        Upper bounds of the grid.
    cell_size : float
        Size of the cell.

    Returns:
    --------
    grid : np.array
        Grid of points.
    '''
    xs = np.linspace(low[0], high[0], np.ceil((high[0] - low[0]) / cell_size).astype(int))
    ys = np.linspace(low[1], high[1], np.ceil((high[1] - low[1]) / cell_size).astype(int))
    grid = np.pad(np.stack(np.meshgrid(xs, ys), axis=-1).reshape(-1, 2), ((0, 0), (0, 1)))
    return grid

def fill_grid(grid, map_data, cell_size=0.25):
    obstacle_grid = np.zeros_like(grid, dtype=bool)
    path_grid = np.zeros_like(grid)
    print(obstacle_grid.shape)
    print(len(map_data.barriers_list))

    for i in range(grid.shape[0]):
        for obstacle in map_data.barriers_list:
            if obstacle.line.contains(Point(grid[i][:2])):
                obstacle_grid[i] = True
                break

    print(sum(obstacle_grid))

    paths = np.pad(split_ways(points, map_data.footways_list, cell_size))
    max_path_dist = 1
    neighbor_cost = 'quadratic'
    tmp, mask = points_near_ref(grid, paths, max_path_dist)
    if neighbor_cost == 'linear':
        pass
    elif neighbor_cost == 'quadratic':
        tmp[:, 3] = path_grid[:, 3] ** 2
    elif neighbor_cost == 'zero':
        tmp[:, 3] = 0
    else:
        print(f'Unknown neighbor cost: {neighbor_cost}')
    tmp[:, 3] /= max_path_dist ** 2 if neighbor_cost == 'quadratic' else 1
    path_grid[mask] = tmp[:, 3]

    grid = path_grid.copy()
    grid[obstacle_grid] = np.inf

    return grid

def points_near_ref(points, reference, max_dist=1):
    '''
    Get points near reference points and set linear distance as cost.

    Parameters:
    -----------
    points : np.array
        Points to check.
    reference : np.array
        Reference points.
    max_dist : float
        Maximum distance to check.

    Returns:
    --------
    points : np.array
        All points with a cost based on distance to reference points.
    '''
    if not isinstance(points, np.ndarray):
        points = np.array(points)
    if not isinstance(reference, np.ndarray):
        reference = np.array(reference)

    tree = cKDTree(reference, compact_nodes=False, balanced_tree=False)
    dists, _ = np.array(tree.query(points, distance_upper_bound=max_dist))
    mask = dists < max_dist
    points = points[mask]
    dists = dists[mask]
    points = np.hstack([points, (dists / max_dist).reshape(-1, 1)])
    return points, mask

def split_ways(points, ways, max_dist=0.25):
    '''
    Equidistantly split ways into points with a maximal step size. Also only use footways from map data,
    as we are not allowed to leave the footways.

    Parameters:
    -----------
    points : dict
        Points to split ways on.
    ways : dict
        Ways to split.
    max_dist : float
        Maximal step size.

    Returns:
    --------
    waypoints : np.array
        Waypoints created from the ways.
    '''
    waypoints = []
    for way in ways:
        for i, (n0, n1) in enumerate(zip(way.nodes, way.nodes[1:])):
            point0 = points[n0.id].ravel()[:2]
            point1 = points[n1.id].ravel()[:2]

            if i == 0:
                waypoints.append(point0)

            dist = np.linalg.norm(point1 - point0)

            if dist <= 1e-3:
                waypoints.append(point1)
                continue

            vec = (point1 - point0) / dist
            num = int(np.ceil(dist / max_dist))
            step = dist / num
            for j in range(num):
                waypoints.append(point0 + (j + 1) * step * vec)

    return np.array(waypoints)

if __name__ == "__main__":

    with open(os.path.join(os.path.dirname(__file__), "../data", 'coords.mapdata'), "rb") as fh:
        map_data = pickle.load(fh)

    low = (map_data.min_x, map_data.min_y)
    high = (map_data.max_x, map_data.max_y)
    grid = create_grid(low, high, cell_size=1)
    grid = fill_grid(grid, map_data, cell_size=1)

