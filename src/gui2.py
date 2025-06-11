import tkinter as tk
from tkinter import messagebox
import numpy as np
from ortools.constraint_solver import pywrapcp, routing_enums_pb2


class TSPGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Shortest Path Solver with Start and Goal")
        self.canvas = tk.Canvas(root, width=600, height=600, bg="white")
        self.canvas.pack(pady=10)

        # Bind left-click to add points
        self.canvas.bind("<Button-1>", self.add_point)

        # Buttons
        self.control_frame = tk.Frame(root)
        self.control_frame.pack(pady=5)
        self.solve_button = tk.Button(self.control_frame, text="Solve Path", command=self.solve_path)
        self.solve_button.pack(side=tk.LEFT, padx=10)
        self.clear_button = tk.Button(self.control_frame, text="Clear", command=self.clear_canvas)
        self.clear_button.pack(side=tk.LEFT, padx=10)

        # Initialize points and state
        self.points = []  # List of (x, y, point_id, type) where type is 'start', 'goal', or 'intermediate'
        self.click_count = 0
        self.point_radius = 5
        self.point_counter = 0

    def add_point(self, event):
        x, y = event.x, event.y
        point_id = self.point_counter
        self.point_counter += 1

        if self.click_count == 0:
            # First click: Start point (green)
            self.points.append((x, y, point_id, "start"))
            self.canvas.create_oval(
                x - self.point_radius, y - self.point_radius,
                x + self.point_radius, y + self.point_radius,
                fill="green", tags=f"point_{point_id}"
            )
            self.canvas.create_text(x + 15, y, text=str(point_id), tags=f"label_{point_id}")
            self.click_count += 1
        elif self.click_count == 1:
            # Second click: Goal point (red)
            self.points.append((x, y, point_id, "goal"))
            self.canvas.create_oval(
                x - self.point_radius, y - self.point_radius,
                x + self.point_radius, y + self.point_radius,
                fill="red", tags=f"point_{point_id}"
            )
            self.canvas.create_text(x + 15, y, text=str(point_id), tags=f"label_{point_id}")
            self.click_count += 1
        else:
            # Subsequent clicks: Intermediate points (blue)
            self.points.append((x, y, point_id, "intermediate"))
            self.canvas.create_oval(
                x - self.point_radius, y - self.point_radius,
                x + self.point_radius, y + self.point_radius,
                fill="blue", tags=f"point_{point_id}"
            )
            self.canvas.create_text(x + 15, y, text=str(point_id), tags=f"label_{point_id}")
            self.click_count += 1

    def compute_distance_matrix(self):
        # Convert points to numpy array
        points_array = np.array([(p[0], p[1]) for p in self.points])
        # Compute Euclidean distance matrix
        n = len(self.points)
        distance_matrix = np.zeros((n, n), dtype=np.int64)
        for i in range(n):
            for j in range(n):
                if i != j:
                    distance_matrix[i, j] = int(
                        np.linalg.norm(points_array[i] - points_array[j]) * 1000)  # Scale for OR-Tools
        return distance_matrix, points_array

    def solve_path(self):
        if len(self.points) < 2:
            messagebox.showwarning("Warning", "Please add at least 2 points.")
            return
        if self.click_count < 2:
            messagebox.showwarning("Warning", "Please add both start and goal points.")
            return
        start_point = next(p for p in self.points if p[3] == "start")
        goal_point = next(p for p in self.points if p[3] == "goal")
        if start_point[0:2] == goal_point[0:2] and len(self.points) == 2:
            messagebox.showwarning("Warning", "Start and goal cannot be the same without intermediate points.")
            return

        # Compute distance matrix
        distance_matrix, points_array = self.compute_distance_matrix()

        # Find start and goal indices
        start_idx = next(i for i, p in enumerate(self.points) if p[3] == "start")
        goal_idx = next(i for i, p in enumerate(self.points) if p[3] == "goal")

        # Set up OR-Tools routing model
        manager = pywrapcp.RoutingIndexManager(len(self.points), 1, [start_idx], [goal_idx])
        routing = pywrapcp.RoutingModel(manager)

        # Define distance callback
        def distance_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return distance_matrix[from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Set search parameters
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )
        search_parameters.time_limit.seconds = 2

        # Solve the problem
        solution = routing.SolveWithParameters(search_parameters)
        if not solution:
            messagebox.showwarning("Warning", "No solution found. Try adjusting points or increasing time limit.")
            return

        # Extract the path (corrected)
        path = []
        index = routing.Start(0)
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            path.append(node)
            index = solution.Value(routing.NextVar(index))
        path.append(manager.IndexToNode(index))  # Include the end node

        # Verify all points are included
        if len(set(path)) != len(self.points):
            messagebox.showwarning("Warning", "Solution does not include all points. Try adjusting points.")
            return

        # Clear previous path
        self.canvas.delete("path")

        # Draw the path
        actual_distance = 0
        for i in range(len(path) - 1):
            start_idx = path[i]
            end_idx = path[i + 1]
            start_x, start_y = self.points[start_idx][0], self.points[start_idx][1]
            end_x, end_y = self.points[end_idx][0], self.points[end_idx][1]
            self.canvas.create_line(start_x, start_y, end_x, end_y, fill="red", width=2, tags="path")
            actual_distance += np.linalg.norm(points_array[start_idx] - points_array[end_idx])

        # Display total distance
        messagebox.showinfo("Path Solution", f"Total distance: {actual_distance:.2f} units")

    def clear_canvas(self):
        self.points = []
        self.click_count = 0
        self.point_counter = 0
        self.canvas.delete("all")


if __name__ == "__main__":
    root = tk.Tk()
    app = TSPGUI(root)
    root.mainloop()