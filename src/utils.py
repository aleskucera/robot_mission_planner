import os

import utm
import yaml
import gpxpy
import numpy as np
from shapely.geometry import Polygon


def parse_path(path_file):
    """
    Parse a path from a GPX file.
    Parameters:
    -----------
    path_file : str
        Path to the GPX file.
    Returns:
    --------
    waypoints : list
        List of waypoints as tuples (latitude, longitude).
    """
    if not path_file:
        print("No path file provided.")
        return []
    if not os.path.exists(path_file):
        print(f"Path file {path_file} does not exist.")
        return []

    if path_file.endswith(".gpx"):
        return np.array(parse_gpx_file(path_file))
    elif path_file.endswith(".yaml"):
        return np.array(parse_yaml_file(path_file))
    else:
        print(f"Unsupported file format: {path_file}.")
        return []


def parse_gpx_file(gpx_file):
    waypoints = []
    try:
        with open(gpx_file, "r") as file:
            gpx = gpxpy.parse(file)
        for waypoint in gpx.waypoints:
            point = {
                "lat": waypoint.latitude,
                "lon": waypoint.longitude,
                "ele": waypoint.elevation or 0,
            }
            waypoints.append(convert_waypoint(point))
    except Exception as e:
        print(f"Error parsing GPX file: {e}")
        return []
    if not waypoints:
        print("No waypoints found in GPX file.")
    else:
        print(f"Parsed {len(waypoints)} waypoints from GPX file.")

    return waypoints


def parse_yaml_file(yaml_file):
    waypoints = []
    with open(yaml_file, "r") as f:
        file_waypoints = yaml.safe_load(f)["waypoints"]
    for waypoint in file_waypoints:
        point = {"lat": waypoint["latitude"], "lon": waypoint["longitude"]}
        if "elevation" in waypoint:
            point["ele"] = waypoint["elevation"]
        else:
            point["ele"] = 0
        waypoints.append(convert_waypoint(point))
    if not waypoints:
        print("No waypoints found in YAML file.")
    else:
        print(f"Parsed {len(waypoints)} waypoints from YAML file.")


def convert_waypoint(point):
    utm_point = utm.from_latlon(point["lat"], point["lon"])[:2]
    return utm_point + (
        point.get("ele", 0),
    )  # Add elevation if available, default to 0


def ways_to_shapely(ways):
    """
    Convert a list of ways to Shapely polygons.
    Parameters:
    ----------
    ways : list
        List of ways, where each way is a list of points (tuples).
    Returns:
    -------
    obstacles : list
        List of Shapely polygons representing the obstacles.
    """
    obstacles = []
    for way in ways:
        obstacles.append(way.line)
    return obstacles
