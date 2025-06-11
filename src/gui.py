import tkinter as tk
import numpy as np
from python_tsp.heuristics import solve_tsp_simulated_annealing

MAX_PROCESSING_TIME = 2  # seconds

class TSPGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Traveling Salesman Problem Solver")
        self.points = []
        self.canvas = tk.Canvas(root, width=600, height=600, bg="white")
        self.canvas.pack(pady=10)

        # Bind left-click to add points
        self.canvas.bind("<Button-1>", self.add_point)

        # Buttons
        self.solve_button = tk.Button(root, text="Solve TSP", command=self.solve_tsp)
        self.solve_button.pack(side=tk.LEFT, padx=10)
        self.clear_button = tk.Button(root, text="Clear", command=self.clear_canvas)
        self.clear_button.pack(side=tk.LEFT, padx=10)

        # Point radius for drawing
        self.point_radius = 5

    def add_point(self, event):
        # Add point where user clicks
        x, y = event.x, event.y
        self.points.append((x, y))
        # Draw point on canvas
        self.canvas.create_oval(
            x - self.point_radius, y - self.point_radius,
            x + self.point_radius, y + self.point_radius,
            fill="blue"
        )

    def compute_distance_matrix(self):
        # Convert points to numpy array
        points_array = np.array(self.points)
        # Compute Euclidean distance matrix
        n = len(self.points)
        distance_matrix = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                distance_matrix[i, j] = np.linalg.norm(points_array[i] - points_array[j])
        return distance_matrix

    def solve_tsp(self):
        if len(self.points) < 2:
            tk.messagebox.showwarning("Warning", "Please add at least 2 points.")
            return

        # Compute distance matrix
        distance_matrix = self.compute_distance_matrix()

        # Solve TSP using python-tsp
        permutation, distance = solve_tsp_simulated_annealing(distance_matrix, max_processing_time=MAX_PROCESSING_TIME)

        # Clear previous path (if any)
        self.canvas.delete("path")

        # Draw the TSP path
        for i in range(len(permutation)):
            start_idx = permutation[i]
            end_idx = permutation[(i + 1) % len(permutation)]  # Connect back to start
            start_x, start_y = self.points[start_idx]
            end_x, end_y = self.points[end_idx]
            self.canvas.create_line(start_x, start_y, end_x, end_y, fill="red", width=2, tags="path")

        # Display total distance
        tk.messagebox.showinfo("TSP Solution", f"Total distance: {distance:.2f} units")

    def clear_canvas(self):
        # Clear points and canvas
        self.points = []
        self.canvas.delete("all")


if __name__ == "__main__":
    root = tk.Tk()
    app = TSPGUI(root)
    root.mainloop()