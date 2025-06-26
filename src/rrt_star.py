import random

import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional
from shapely.geometry import Point, LineString
from matplotlib.patches import Polygon as MplPolygon


class RRTStar:
    def __init__(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        obstacles: List,
        grid: np.ndarray,
        max_iter: int = 5000,
        step_size: float = 0.5,
        neighbor_radius: float = 1.0,
        grid_scale: float = 1.0,
        traversability_threshold: float = 0.75,
        simplify: bool = True,
    ):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacles = obstacles
        self.grid = grid
        self.grid_shape = grid.shape
        self.grid_scale = grid_scale  # Grid cell size in world coordinates
        self.max_iter = max_iter
        self.step_size = step_size
        self.neighbor_radius = neighbor_radius
        self.nodes = [self.start]
        self.parent = {0: None}
        self.cost = {0: 0.0}
        self.goal_tolerance = step_size / 2
        self.traversability_threshold = traversability_threshold
        self.simplify = simplify

    def _is_collision(
        self, point1: np.ndarray, point2: Optional[np.ndarray] = None
    ) -> bool:
        """Check for collision between obstacles and map segments."""
        if point2 is None:
            geom = Point(point1)
        else:
            geom = LineString([point1, point2])

        for obstacle in self.obstacles:
            if obstacle.contains(geom) or obstacle.intersects(geom):
                return True
        if point2 is not None:
            p1_grid = (point1[0] / self.grid_scale, point1[1] / self.grid_scale)
            p2_grid = (point2[0] / self.grid_scale, point2[1] / self.grid_scale)
            p1_grid = (int(p1_grid[0]), int(p1_grid[1]))
            p2_grid = (int(p2_grid[0]), int(p2_grid[1]))
            bres_line = self._bresenham(p1_grid, p2_grid)
            for point in bres_line:
                if (
                    0 < point[0] < self.grid_shape[1]
                    and 0 < point[1] < self.grid_shape[0]
                ):
                    break
                if (
                    self.grid[point[0] - 1, point[1] - 1]
                    >= self.traversability_threshold
                ):
                    return True
        return False

    def _bresenham(
        self, start: Tuple[float, float], goal: Tuple[float, float]
    ) -> List[Tuple[float, float]]:
        """Bresenham's line algorithm
        Args:
            start: (float64, float64) - start coordinate
            goal: (float64, float64) - goal coordinate
        Returns:
            interlying points between the start and goal coordinate
        """
        x0, y0 = start
        x1, y1 = goal
        line = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                line.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                line.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        x = goal[0]
        y = goal[1]
        return line

    def _get_grid_cost(self, point: np.ndarray) -> float:
        """Get the traversability cost at a point from the grid."""
        # Convert world coordinates to grid indices
        i = min(int(point[1] / self.grid_scale), self.grid_shape[0] - 1)
        j = min(int(point[0] / self.grid_scale), self.grid_shape[1] - 1)
        i = max(0, i)
        j = max(0, j)
        return float(self.grid[i, j])

    def _sample_point(self) -> np.ndarray:
        """Sample a random point in the grid space, biased towards traversable areas."""
        while True:
            x = random.uniform(0, self.grid_shape[1] * self.grid_scale)
            y = random.uniform(0, self.grid_shape[0] * self.grid_scale)
            point = np.array([x, y])
            if self._get_grid_cost(
                point
            ) < self.traversability_threshold and not self._is_collision(point):
                return point
            # Occasionally allow sampling in non-traversable areas to ensure exploration
            # if random.random() < 0.1:
            #     return point

    def _nearest_node(self, point: np.ndarray) -> int:
        """Find the index of the nearest node to the given point."""
        distances = [np.linalg.norm(node - point) for node in self.nodes]
        return distances.index(min(distances))

    def _steer(self, start: np.ndarray, target: np.ndarray) -> np.ndarray:
        """Steer from start towards target with a fixed step size."""
        direction = target - start
        distance = np.linalg.norm(direction)
        if distance < self.step_size:
            return target
        return start + (direction / distance) * self.step_size

    def _get_near_nodes(self, new_point: np.ndarray) -> List[int]:
        """Find indices of nodes within neighbor_radius of the new point."""
        return [
            i
            for i, node in enumerate(self.nodes)
            if np.linalg.norm(node - new_point) < self.neighbor_radius
            and np.linalg.norm(node - new_point) > 0
        ]

    def _path_cost(self, start: np.ndarray, end: np.ndarray) -> float:
        """Calculate the cost of a path segment using Bresenham's line algorithm."""
        distance = np.linalg.norm(end - start)
        # Convert world coordinates to grid indices for Bresenham
        start_grid = (start[0] / self.grid_scale, start[1] / self.grid_scale)
        end_grid = (end[0] / self.grid_scale, end[1] / self.grid_scale)
        start_grid = (int(start_grid[0]), int(start_grid[1]))
        end_grid = (int(end_grid[0]), int(end_grid[1]))
        # Get grid cells along the line
        cells = self._bresenham(start_grid, end_grid)
        total_cost = 0.0
        valid_cells = 0
        for x, y in cells:
            # Ensure indices are within grid bounds
            if 0 <= y < self.grid_shape[0] and 0 <= x < self.grid_shape[1]:
                total_cost += self.grid[int(y), int(x)]
                valid_cells += 1
        # Average the grid cost over all valid cells
        avg_cost = total_cost / valid_cells if valid_cells > 0 else 0.0
        return distance * (
            1 + avg_cost
        )  # Scale distance by average traversability cost

    def find_path(self) -> Optional[np.ndarray]:
        """Main RRT* algorithm to find a path from start to goal."""
        for _ in range(self.max_iter):
            while True:
                # Occasionally sample the goal to bias exploration
                if random.random() < 0.1:
                    rand_point = self.goal
                else:
                    rand_point = self._sample_point()

                # Find nearest node
                nearest_idx = self._nearest_node(rand_point)
                nearest = self.nodes[nearest_idx]
                new_point = self._steer(nearest, rand_point)

                # Check if new point is valid
                if (
                    not self._is_collision(new_point)
                    or self._get_grid_cost(new_point) < self.traversability_threshold
                ):
                    break

            if not self._is_collision(nearest, new_point):
                new_idx = len(self.nodes)
                self.nodes.append(new_point)
                min_cost = self.cost[nearest_idx] + self._path_cost(nearest, new_point)
                min_parent = nearest_idx

                # Find near nodes for rewiring
                near_nodes = self._get_near_nodes(new_point)
                # Choose best parent
                for idx in near_nodes:
                    node = self.nodes[idx]
                    if not self._is_collision(node, new_point):
                        cost = self.cost[idx] + self._path_cost(node, new_point)
                        if cost < min_cost:
                            min_cost = cost
                            min_parent = idx

                self.parent[new_idx] = min_parent
                self.cost[new_idx] = min_cost

                # Rewire the tree
                for idx in near_nodes:
                    if idx == min_parent:
                        continue
                    node = self.nodes[idx]
                    new_cost = self.cost[new_idx] + self._path_cost(new_point, node)
                    if new_cost < self.cost[idx] and not self._is_collision(
                        new_point, node
                    ):
                        self.parent[idx] = new_idx
                        self.cost[idx] = new_cost

                # Check if we can connect to the goal
                if np.linalg.norm(new_point - self.goal) < self.goal_tolerance:
                    if not self._is_collision(new_point, self.goal):
                        goal_idx = len(self.nodes)
                        self.nodes.append(self.goal)
                        self.parent[goal_idx] = new_idx
                        self.cost[goal_idx] = self.cost[new_idx] + self._path_cost(
                            new_point, self.goal
                        )
                        return np.array(self._reconstruct_path(goal_idx))

        return None

    def _reconstruct_path(self, goal_idx: int) -> List[np.ndarray]:
        """Reconstruct the path from start to goal."""
        path = []
        current_idx = goal_idx
        while current_idx is not None:
            path.append(self.nodes[current_idx])
            current_idx = self.parent[current_idx]
        return self._simplify_path(path[::-1]) if self.simplify else path[::-1]

    def _simplify_path(self, path):
        new_path = [path[0]]
        idx = 1
        while tuple(new_path[-1]) != tuple(path[-1]):
            start = new_path[-1]
            for goal in path[idx:]:
                if not self._is_collision(new_path[-1], goal):
                    start = goal
                    idx += 1
                    if tuple(start) == tuple(path[-1]):
                        new_path.append(start)
                        break
                else:
                    new_path.append(start)
                    break
        return new_path

    def visualize(self, path: Optional[List[np.ndarray]] = None):
        """Visualize the grid, obstacles, RRT* tree, and path using Matplotlib."""
        _, ax = plt.subplots()

        # Plot grid as a heatmap (0: white, 1: gray)
        grid_display = np.flipud(self.grid)  # Flip for correct orientation
        ax.imshow(
            grid_display,
            cmap="Greys",
            origin="lower",
            extent=[
                0,
                self.grid_shape[1] * self.grid_scale,
                0,
                self.grid_shape[0] * self.grid_scale,
            ],
        )

        # Plot obstacles
        for obstacle in self.obstacles:
            if obstacle.geom_type == "Polygon":
                x, y = obstacle.exterior.xy
                ax.add_patch(MplPolygon(list(zip(x, y)), color="red", alpha=0.5))

        # Plot RRT* tree
        for idx, node in enumerate(self.nodes):
            if self.parent[idx] is not None:
                parent_node = self.nodes[self.parent[idx]]
                ax.plot(
                    [node[0], parent_node[0]],
                    [node[1], parent_node[1]],
                    "c-",
                    linewidth=0.5,
                )

        # Plot start and goal
        ax.plot(self.start[0], self.start[1], "go", label="Start")
        ax.plot(self.goal[0], self.goal[1], "bo", label="Goal")

        # Plot path if found
        if path is not None:
            path = np.array(path)
            ax.plot(path[:, 0], path[:, 1], "k-", linewidth=2, label="Path")

        # Set plot properties
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_title("RRT* Path Planning")
        ax.legend()
        ax.grid(True)
        plt.show()


# Example usage
if __name__ == "__main__":
    from shapely.geometry import Polygon

    # Define start and goal points
    start = (0.0, 0.0)
    goal = (10.0, 10.0)

    # Define obstacles as Shapely polygons
    obstacles = [
        Polygon([(2, 2), (2, 4), (4, 4), (4, 2)]),
        Polygon([(6, 6), (6, 8), (8, 8), (8, 6)]),
        Polygon([(3, 7), (3, 9), (5, 9), (5, 7)]),
    ]

    # Define a 10x10 grid with 0-1 traversability costs
    grid = np.zeros((100, 100), dtype=float)
    grid[40:60, 40:60] = 1  # Non-traversable region

    # Initialize and run RRT*
    rrt_star = RRTStar(
        start,
        goal,
        obstacles,
        grid,
        max_iter=2000,
        step_size=0.1,
        neighbor_radius=1.5,
        grid_scale=0.1,
    )
    path = rrt_star.find_path()

    if path is not None:
        print("Path found:")
        for point in path:
            print(f"({point[0]:.2f}, {point[1]:.2f})")
    else:
        print("No path found.")

    rrt_star.visualize(path)
