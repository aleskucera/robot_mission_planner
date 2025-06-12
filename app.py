from flask import Flask, render_template, request, jsonify, Response
import numpy as np
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import subprocess
import tempfile
import os
import re
import select
import time
import threading
import shutil
from uuid import uuid4

app = Flask(__name__)

# Global dictionary to track active wormhole processes
active_transfers = {}


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
                    dlat = np.radians(lat2 - lat1)
                    dlng = np.radians(lng2 - lng1)
                    a = (np.sin(dlat / 2) ** 2 +
                         np.cos(np.radians(lat1)) * np.cos(np.radians(lat2)) *
                         np.sin(dlng / 2) ** 2)
                    c = 2 * np.arcsin(np.sqrt(a))
                    distance = 6371000 * c
                    distance_matrix[i, j] = int(distance)

        return distance_matrix, self.points

    def solve_path(self):
        if len(self.points) < 2:
            return None, "Need at least 2 points"

        start_points = [i for i, p in enumerate(self.points) if p['type'] == 'start']
        goal_points = [i for i, p in enumerate(self.points) if p['type'] == 'goal']

        if not start_points or not goal_points:
            return None, "Need both start and goal points"

        start_idx = start_points[0]
        goal_idx = goal_points[0]

        distance_matrix, points = self.compute_distance_matrix_euclidean()
        if distance_matrix is None:
            return None, "Failed to compute distances"

        manager = pywrapcp.RoutingIndexManager(len(points), 1, [start_idx], [goal_idx])
        routing = pywrapcp.RoutingModel(manager)

        def distance_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return distance_matrix[from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
        )
        search_parameters.time_limit.seconds = 5

        solution = routing.SolveWithParameters(search_parameters)
        if not solution:
            return None, "No solution found"

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

        path.append(manager.IndexToNode(index))

        return path, f"Total distance: {total_distance} meters"

    def clear_points(self):
        self.points = []


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


@app.route('/create_wormhole', methods=['POST'])
def create_wormhole():
    data = request.json
    gpx_data = data.get('gpx')

    if not gpx_data:
        app.logger.error("No GPX data provided")
        return jsonify({'success': False, 'message': 'No GPX data provided'})

    try:
        # Create temporary directory and file
        temp_dir = tempfile.mkdtemp()
        file_path = os.path.join(temp_dir, 'path.gpx')

        # Write GPX data to file
        with open(file_path, 'w') as f:
            f.write(gpx_data)

        # Check if wormhole is installed
        try:
            subprocess.run(["wormhole", "--version"], capture_output=True, text=True, check=True)
        except subprocess.CalledProcessError:
            app.logger.error("Magic-Wormhole is not installed or not working")
            return jsonify({
                'success': False,
                'message': 'Magic-Wormhole is not installed or not working'
            })
        except FileNotFoundError:
            app.logger.error("'wormhole' command not found. Please install magic-wormhole")
            return jsonify({
                'success': False,
                'message': "'wormhole' command not found. Please install magic-wormhole"
            })

        # Generate a unique transfer ID
        transfer_id = str(uuid4())

        # Prepare the wormhole send command
        cmd = ["wormhole", "send", file_path]
        app.logger.info(f"Starting wormhole transfer {transfer_id}: {' '.join(cmd)}")

        # Set environment to force unbuffered output
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"

        # Start the wormhole process
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            env=env
        )

        app.logger.info(f"Process ID for transfer {transfer_id}: {process.pid}")

        # Store process and temp dir in active_transfers
        active_transfers[transfer_id] = {
            'process': process,
            'temp_dir': temp_dir,
            'start_time': time.time(),
            'status': 'running'
        }

        # Function to capture wormhole code in a background thread
        def capture_wormhole_code():
            wormhole_code = None
            errors = []
            try:
                while True:
                    readable, _, _ = select.select([process.stdout, process.stderr], [], [], 0.1)
                    for stream in readable:
                        line = stream.readline().strip()
                        if line:
                            match = re.search(r"Wormhole code is: (\S+)", line)
                            if match:
                                wormhole_code = match.group(1)
                                app.logger.info(f"Wormhole code captured for transfer {transfer_id}: {wormhole_code}")
                            elif stream == process.stderr:
                                errors.append(line)

                    if wormhole_code:
                        active_transfers[transfer_id]['code'] = wormhole_code
                        break

                    if process.poll() is not None:
                        break

                # Wait for process to complete or timeout (60 seconds total)
                timeout = 60
                while process.poll() is None and (time.time() - active_transfers[transfer_id]['start_time']) < timeout:
                    time.sleep(1)

                if process.poll() is None:
                    process.kill()
                    app.logger.warning(f"Transfer {transfer_id} timed out after {timeout} seconds")
                    active_transfers[transfer_id]['status'] = 'timed_out'
                else:
                    active_transfers[transfer_id]['status'] = 'completed' if process.returncode == 0 else 'failed'

                # Clean up
                try:
                    shutil.rmtree(temp_dir)
                    app.logger.info(f"Cleaned up temp directory for transfer {transfer_id}")
                except Exception as e:
                    app.logger.error(f"Error cleaning up temp directory for transfer {transfer_id}: {str(e)}")

                if not wormhole_code:
                    active_transfers[transfer_id]['status'] = 'failed'
                    app.logger.error(f"No wormhole code captured for transfer {transfer_id}: {errors}")
            except Exception as e:
                app.logger.error(f"Error in wormhole thread for transfer {transfer_id}: {str(e)}")
                active_transfers[transfer_id]['status'] = 'failed'
                try:
                    process.kill()
                except:
                    pass

        # Start background thread
        threading.Thread(target=capture_wormhole_code, daemon=True).start()

        # Wait briefly for the code to be captured
        start_time = time.time()
        while time.time() - start_time < 10:  # 10-second timeout for code capture
            if transfer_id in active_transfers and 'code' in active_transfers[transfer_id]:
                wormhole_code = active_transfers[transfer_id]['code']
                return jsonify({
                    'success': True,
                    'code': wormhole_code,
                    'transfer_id': transfer_id
                })
            time.sleep(0.1)

        # If code not captured, return error
        process.kill()
        try:
            shutil.rmtree(temp_dir)
        except:
            pass
        active_transfers.pop(transfer_id, None)
        app.logger.error(f"Failed to capture wormhole code for transfer {transfer_id} within 10 seconds")
        return jsonify({
            'success': False,
            'message': 'Failed to capture wormhole code within 10 seconds',
            'details': {'command': ' '.join(cmd), 'pid': process.pid}
        })

    except Exception as e:
        import traceback
        app.logger.error(f"Error in create_wormhole: {str(e)}")
        app.logger.error(traceback.format_exc())
        try:
            process.kill()
        except:
            pass
        try:
            shutil.rmtree(temp_dir)
        except:
            pass
        active_transfers.pop(transfer_id, None)
        return jsonify({
            'success': False,
            'message': f'Error creating wormhole: {str(e)}',
            'traceback': traceback.format_exc()
        })


@app.route('/cancel_wormhole', methods=['POST'])
def cancel_wormhole():
    data = request.json
    transfer_id = data.get('transfer_id')

    if not transfer_id or transfer_id not in active_transfers:
        return jsonify({
            'success': False,
            'message': 'Invalid or unknown transfer ID'
        })

    try:
        transfer = active_transfers[transfer_id]
        process = transfer['process']
        temp_dir = transfer['temp_dir']

        if process.poll() is None:
            process.kill()
            app.logger.info(f"Cancelled wormhole transfer {transfer_id}")

        try:
            shutil.rmtree(temp_dir)
            app.logger.info(f"Cleaned up temp directory for transfer {transfer_id}")
        except Exception as e:
            app.logger.error(f"Error cleaning up temp directory for transfer {transfer_id}: {str(e)}")

        active_transfers[transfer_id]['status'] = 'cancelled'
        active_transfers.pop(transfer_id)
        return jsonify({
            'success': True,
            'message': 'Wormhole transfer cancelled'
        })

    except Exception as e:
        app.logger.error(f"Error cancelling wormhole transfer {transfer_id}: {str(e)}")
        return jsonify({
            'success': False,
            'message': f'Error cancelling wormhole transfer: {str(e)}'
        })


if __name__ == '__main__':
    app.run(debug=True)