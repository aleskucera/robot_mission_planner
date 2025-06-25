import os
import yaml
import pickle
import argparse

import numpy as np
import shapely.geometry as sg
from scipy.spatial import cKDTree
from shapely.geometry import LineString

from rrt_star import RRTStar
from map_data import MapData, CoordsData


class ReplanPath:
    def __init__(self, args):
        self.args = args

        self.grid = self._create_grid(args.low, args.high, args.cell_size)

    def replan_rrt(self, path, obstacles, grid):
        new_path = []
        for i in range(len(path) - 1):
            new_path.append(path[i])
            start = path[i]
            goal = path[i + 1]
            path_seg = LineString([start[:2], goal[:2]])
            if self._colides(path_seg, obstacles):
                way = rrt(start[:2], goal[:2], obstacles, grid)
                new_path.extend(way[1:-1])
                break

        new_path.append(path[-1])
        return new_path

    def rrt(self, start, goal, obstacles, grid):
        rrt_star = RRTStar(
            start, goal, obstacles, grid, simplify=self.args.simplify_path
        )
        path = rrt_star.find_path()

        return path

    def _colides(self, path_seg, obstacles):
        for obstacle in obstacles:
            if sg.intersects(path_seg, obstacle):
                return True
        return False

    def _create_grid(self, low, high, cell_size=0.25):
        """
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
        """
        xs = np.linspace(
            low[0], high[0], np.ceil((high[0] - low[0]) / cell_size).astype(int)
        )
        ys = np.linspace(
            low[1], high[1], np.ceil((high[1] - low[1]) / cell_size).astype(int)
        )
        grid = np.pad(
            np.stack(np.meshgrid(xs, ys), axis=-1).reshape(-1, 2), ((0, 0), (0, 1))
        )
        return grid

    def fill_grid(self, map_data):
        points = map_data.get_points()
        path_grid = np.zeros_like(self.grid)

        paths = np.pad(
            self._split_ways(points, map_data.footways_list, self.args.cell_size),
            ((0, 0), (0, 1)),
        )
        max_path_dist = 1
        neighbor_cost = "quadratic"
        print(path_grid.shape, paths.shape)
        tmp, mask = self._points_near_ref(path_grid, paths, max_path_dist)
        print(tmp.shape, mask.shape)
        print(mask.sum())
        path_grid = np.pad(path_grid, ((0, 0), (0, 1)))
        if neighbor_cost == "linear":
            pass
        elif neighbor_cost == "quadratic":
            tmp[:, 3] = path_grid[:, 3] ** 2
        elif neighbor_cost == "zero":
            tmp[:, 3] = 0
        else:
            print(f"Unknown neighbor cost: {neighbor_cost}")
        tmp[:, 3] /= max_path_dist**2 if neighbor_cost == "quadratic" else 1
        path_grid[mask] = tmp[:, 3]

        return path_grid.copy()

    def _points_near_ref(self, points, reference, max_dist=1):
        """
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
        """
        if not isinstance(points, np.ndarray):
            points = np.array(points)
        if not isinstance(reference, np.ndarray):
            reference = np.array(reference)

        tree = cKDTree(reference, compact_nodes=False, balanced_tree=False)
        dists, _ = np.array(tree.query(points, distance_upper_bound=max_dist))
        mask = dists < max_dist
        points = points[mask]
        dists = dists[mask]

        return (np.hstack([points, (dists / max_dist).reshape(-1, 1)]), mask)

    def _split_ways(self, points, ways, max_dist=0.25):
        """
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
        """
        waypoints = []
        for way in ways:
            for i, (n0, n1) in enumerate(zip(way.nodes, way.nodes[1:])):
                point0 = points[n0.id].ravel()[:2]
                point1 = points[n1.id].ravel()[:2]
                dist = np.linalg.norm(point1 - point0)

                if i == 0:
                    waypoints.append(point0)
                if dist <= 1e-3:
                    waypoints.append(point1)
                    continue

                vec = (point1 - point0) / dist
                num = int(np.ceil(dist / max_dist))
                step = dist / num
                for j in range(num):
                    waypoints.append(point0 + (j + 1) * step * vec)

        return np.array(waypoints)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--file", type=str, default="coords.mapdata", help="Map data file"
    )
    parser.add_argument(
        "--cell_size", type=float, default=0.25, help="Cell size for the grid"
    )

    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    with open(
        os.path.join(os.path.dirname(__file__), "../data", args.file), "rb"
    ) as fh:
        map_data = pickle.load(fh)

    args.low = (map_data.min_x, map_data.min_y)
    args.high = (map_data.max_x, map_data.max_y)

    replaner = ReplanPath(args)
    replaner.fill_grid(map_data)
