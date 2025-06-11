from flask import Flask, render_template, request, jsonify
import numpy as np
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import requests
import json

app = Flask(__name__)


class PathSolver:
    def __init__(self):
        self.points = []

    def add_point(self, lat, lng, point_type):
        point_id = len(self.points)
        self.points.append({
            'id': point_id,
            'lat': lat,
            'lng': lng,
            'type': point_type
        })
        return point_id

    def compute_distance_matrix_euclidean(self):
        n = len(self.points)
        distance_matrix = np.zeros((n, n), dtype=np.int64)

        for i in range(n):
            for j in range(n):
                if i != j:
                    lat1, lng1 = self.points[i]['lat'], self.points[i]['lng']
                    lat2, lng2 = self.points[j]['lat'], self.points[j]['lng']

                    # Haversine distance approximation (simplified)
                    dlat = np.radians(lat2 - lat1)
                    dlng = np.radians(lng2 - lng1)
                    a = (np.sin(dlat / 2) ** 2 +
                         np.cos(np.radians(lat1)) * np.cos(np.radians(lat2)) *
                         np.sin(dlng / 2) ** 2)
                    c = 2 * np.arcsin(np.sqrt(a))
                    distance = 6371000 * c  # Earth radius in meters
                    distance_matrix[i, j] = int(distance)

        return distance_matrix, self.points

    def solve_path(self):
        if len(self.points) < 2:
            return None, "Need at least 2 points"

        # Find start and goal points
        start_points = [i for i, p in enumerate(self.points) if p['type'] == 'start']
        goal_points = [i for i, p in enumerate(self.points) if p['type'] == 'goal']

        if not start_points or not goal_points:
            return None, "Need both start and goal points"

        start_idx = start_points[0]
        goal_idx = goal_points[0]

        # Compute distance matrix
        distance_matrix, points = self.compute_distance_matrix_euclidean()
        if distance_matrix is None:
            return None, "Failed to compute distances"

        # Set up OR-Tools routing model
        manager = pywrapcp.RoutingIndexManager(len(points), 1, [start_idx], [goal_idx])
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
        search_parameters.time_limit.seconds = 5

        # Solve the problem
        solution = routing.SolveWithParameters(search_parameters)
        if not solution:
            return None, "No solution found"

        # Extract the path
        path = []
        index = routing.Start(0)
        total_distance = 0

        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            path.append(node)
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            if not routing.IsEnd(index):
                total_distance += routing.GetArcCostForVehicle(previous_index, index, 0)

        # Add the final node
        path.append(manager.IndexToNode(index))

        return path, f"Total distance: {total_distance} meters"

    def clear_points(self):
        self.points = []


# Global solver instance
solver = PathSolver()

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/add_point', methods=['POST'])
def add_point():
    data = request.json
    lat = data['lat']
    lng = data['lng']
    point_type = data['type']

    point_id = solver.add_point(lat, lng, point_type)

    return jsonify({
        'success': True,
        'point_id': point_id,
        'total_points': len(solver.points)
    })


@app.route('/solve_path', methods=['POST'])
def solve_path():
    path, message = solver.solve_path()

    if path is None:
        return jsonify({
            'success': False,
            'message': message
        })

    # Convert path indices to coordinates
    path_coords = []
    for idx in path:
        point = solver.points[idx]
        path_coords.append({
            'lat': point['lat'],
            'lng': point['lng'],
            'type': point['type'],
            'id': point['id']
        })

    return jsonify({
        'success': True,
        'path': path_coords,
        'message': message
    })


@app.route('/clear_points', methods=['POST'])
def clear_points():
    solver.clear_points()
    return jsonify({'success': True})


@app.route('/get_points', methods=['GET'])
def get_points():
    return jsonify({'points': solver.points})


if __name__ == '__main__':
    app.run(debug=True)
