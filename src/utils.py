import os

import utm
import yaml
import gpxpy
import numpy as np


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
        return parse_gpx_file(path_file)
    elif path_file.endswith(".yaml"):
        return parse_yaml_file(path_file)
    else:
        print(f"Unsupported file format: {path_file}.")
        return []


def parse_gpx_file(gpx_file):
    waypoints = []
    zone_num, zone_let = None, None
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
        zone_num, zone_let = utm.from_latlon(
            gpx.waypoints[0].latitude, gpx.waypoints[0].longitude
        )[2:]

    return np.array(waypoints), zone_num, zone_let


def parse_yaml_file(yaml_file):
    waypoints = []
    zone_num, zone_let = None, None
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
        zone_num, zone_let = utm.from_latlon(
            file_waypoints[0]["latitude"], file_waypoints[0]["longitude"]
        )[2:]

    return np.array(waypoints), zone_num, zone_let


def convert_waypoint(point):
    utm_point = utm.from_latlon(point["lat"], point["lon"])[:2]
    return utm_point + (
        point.get("ele", 0),
    )  # Add elevation if available, default to 0


def utm_path_to_latlon(path, zone_num, zone_let):
    wgs_path = []
    for point in path:
        lat, lon = utm.to_latlon(point[0], point[1], zone_num, zone_let)
        wgs_path.append({"latitude": lat, "longitude": lon, "elevation": point[2]})
    return wgs_path


def create_gpx_content(waypoints_data, creator_name="YAML to GPX Converter"):
    """
    Generates the XML content for a GPX file from a list of waypoint dictionaries.
    """
    gpx_waypoints = []
    for point in waypoints_data:
        try:
            # Create a <wpt> tag for each point
            lat = point["latitude"]
            lon = point["longitude"]
            gpx_waypoints.append(f'  <wpt lat="{lat}" lon="{lon}"></wpt>')
        except KeyError as e:
            print(
                f"Warning: Skipping a waypoint due to missing key: {e}", file=sys.stderr
            )
            continue

    # Join all waypoint strings
    waypoints_xml = "\n".join(gpx_waypoints)

    # Assemble the final GPX file content using a template
    gpx_template = f"""<?xml version="1.0" encoding="UTF-8"?>
<gpx xmlns="http://www.topografix.com/GPX/1/1" version="1.1" creator="{creator_name}">
{waypoints_xml}
</gpx>
    """
    return gpx_template.strip()


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
