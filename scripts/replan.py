#!/usr/bin/env python3

import os
import sys
import pickle
import argparse

import numpy as np
import shapely as sh
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from shapely.geometry import LineString
from matplotlib.patches import Polygon as MplPolygon

sys.path.append(f"{os.path.dirname(__file__)}/../src")
from rrt_star import RRTStar
from map_data import MapData, CoordsData
from utils import parse_path, ways_to_shapely, create_gpx_content, utm_path_to_latlon


class ReplanPath:
    def __init__(self, args, obstacles=None):
        self.args = args

        self.grid = self._create_grid(args.low, args.high, args.cell_size)
        self.obstacles = obstacles

    def replan_rrt(self, path):
        new_path = []
        for i in range(len(path) - 1):
            new_path.append(path[i])
            start = path[i]
            goal = path[i + 1]
            path_seg = LineString([start[:2], goal[:2]])
            if self._colides(path_seg, self.obstacles):
                way = self._rrt(
                    start[:2] - self.args.low, goal[:2] - self.args.low, self.obstacles
                )
                if way is None:
                    print("RRT* failed to find a path.")
                    return None
                new_path.extend(way[1:-1])

        new_path.append(path[-1])
        return np.array(new_path)

    def _reshape_grid(self):
        """
        Reshape the grid to match the shape of the map data.
        """
        low = self.args.low
        high = self.args.high
        cell_size = self.args.cell_size

        num_x = int(np.ceil((high[0] - low[0]) / cell_size))
        num_y = int(np.ceil((high[1] - low[1]) / cell_size))

        grid = np.zeros((num_x, num_y), dtype=np.float32)
        x = self.grid[:, 0]
        y = self.grid[:, 1]
        c = self.grid[:, 3]

        x_indices = np.floor((x - low[0]) / cell_size).astype(int)
        y_indices = np.floor((y - low[1]) / cell_size).astype(int)
        x_indices = np.clip(x_indices, 0, num_x - 1)
        y_indices = np.clip(y_indices, 0, num_y - 1)

        grid[x_indices, y_indices] = c
        return grid.T

    def _convert_obstacles(self, obstacles):
        """
        Convert obstacles to a format suitable for RRT*.
        Parameters:
        -----------
        obstacles : list
            List of obstacles as shapely geometries.
        Returns:
        --------
        obst : list
            List of obstacles as shapely polygons.
        """
        obst = []
        for obstacle in obstacles:
            obstacle = sh.transform(obstacle, lambda x: (x - self.args.low))
            obst.append(obstacle)
        return obst

    def _rrt(self, start, goal, obstacles):
        grid = self._reshape_grid()
        obst = self._convert_obstacles(obstacles)
        self.obstacles = obstacles
        rrt_star = RRTStar(start, goal, obst, grid, simplify=self.args.simplify_path)
        path = rrt_star.find_path()
        if path is None:  # debug
            rrt_star.visualize()
        else:
            path += self.args.low  # Convert back to original coordinates
            path = np.hstack([path, np.zeros((path.shape[0], 1))])  # Add z-coordinate

        return path

    def _colides(self, path_seg, obstacles):
        for obstacle in obstacles:
            if obstacle.contains(path_seg) or obstacle.intersects(path_seg):
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
        path_grid = self.grid

        paths = np.pad(
            self._split_ways(points, map_data.footways_list, self.args.cell_size),
            ((0, 0), (0, 1)),
        )
        max_path_dist = 1
        neighbor_cost = "quadratic"
        tmp, mask = self._points_near_ref(path_grid, paths, max_path_dist)
        path_grid = np.pad(path_grid, ((0, 0), (0, 1)))
        if neighbor_cost == "linear":
            pass
        elif neighbor_cost == "quadratic":
            tmp[:, 3] = tmp[:, 3] ** 2
        elif neighbor_cost == "zero":
            tmp[:, 3] = 0
        else:
            print(f"Unknown neighbor cost: {neighbor_cost}")
        tmp[:, 3] /= max_path_dist**2 if neighbor_cost == "quadratic" else 1
        path_grid[mask, 3] = tmp[:, 3]
        path_grid[~mask, 3] = 0.5

        self.grid = path_grid

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

    def visualize(self, path, old_path=None):
        """Visualize the grid, obstacles, RRT* tree, and path using Matplotlib."""
        _, ax = plt.subplots()

        # Plot grid as a heatmap (0: white, 1: gray)
        grid_display = self._reshape_grid()
        ax.imshow(
            grid_display,
            cmap="Greys",
            origin="lower",
            extent=[
                self.args.low[0],
                self.args.high[0],
                self.args.low[1],
                self.args.high[1],
            ],
        )

        # Plot obstacles
        for obstacle in self.obstacles:
            if obstacle.geom_type == "Polygon":
                x, y = obstacle.exterior.xy
                ax.add_patch(MplPolygon(list(zip(x, y)), color="red", alpha=0.5))

        # Plot old path if provided
        if old_path is not None:
            # old_path = np.array(old_path)
            ax.plot(old_path[:, 0], old_path[:, 1], "c-", linewidth=2, label="Path")

        # Plot path if found
        if path is not None:
            # path = np.array(path)
            ax.plot(path[:, 0], path[:, 1], "m-", linewidth=2, label="Path")
            ax.scatter(path[:, 0], path[:, 1], c="m", s=20, label="Path Points")

        # Plot start and goal
        ax.plot(path[0, 0], path[0, 1], "go", label="Start")
        ax.plot(path[-1, 0], path[-1, 1], "bo", label="Goal")

        # Set plot properties
        ax.set_xlabel("Northing [m]")
        ax.set_ylabel("Easting [m]")
        ax.set_title("Replanned Path")

        ax.legend()
        ax.grid(True)
        ax.set_aspect("equal")
        ax.set_xlim(self.args.low[0], self.args.high[0])
        ax.set_ylim(self.args.low[1], self.args.high[1])

        plt.show()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", type=str, default="data/coords.gpx", help="Path file")
    parser.add_argument("--file", type=str, default=None, help="Map data file")
    parser.add_argument("--simplify_path", action="store_true", help="Simplify path")
    parser.add_argument(
        "--cell_size", type=float, default=0.25, help="Cell size for the grid"
    )
    parser.add_argument("--save", type=str, default=None, help="Save path to file")
    parser.add_argument("--visualize", action="store_true", help="Visualize the path")

    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    path_file = os.path.join(os.path.dirname(__file__), "../", args.path)
    path_data = parse_path(path_file)

    if args.file is None:
        map_data = MapData(path_data, coords_type="array")
        map_data.run_queries()
        map_data.run_parse()
    else:
        with open(
            os.path.join(os.path.dirname(__file__), "../", args.file), "rb"
        ) as fh:
            map_data = pickle.load(fh)

    args.low = (map_data.min_x, map_data.min_y)
    args.high = (map_data.max_x, map_data.max_y)
    obstacles = ways_to_shapely(map_data.barriers_list)

    replanner = ReplanPath(args, obstacles)
    replanner.fill_grid(map_data)
    new_path = replanner.replan_rrt(path_data[0])

    if args.save:
        new_wgs_path = utm_path_to_latlon(new_path, path_data[1], path_data[2])
        gpx_content = create_gpx_content(new_wgs_path, creator_name="RRT* Replanner")
        with open(args.save, "w") as f:
            f.write(gpx_content)

    if args.visualize:
        replanner.visualize(new_path, path_data[0])
