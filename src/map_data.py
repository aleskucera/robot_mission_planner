try:
    import cPickle as pickle
except ImportError:
    import pickle

try:
    import rospy
except ImportError:
    import time

    class rospy:
        def loginfo(msg):
            print("INFO: ", msg)

        def logwarn(msg):
            print("\033[93mWARN:  {}\033[00m".format(msg))

        def logerr(msg):
            print("\033[91mERROR: {}\033[00m".format(msg))

        def Duration(seconds):
            return float(seconds)

        def sleep(seconds):
            time.sleep(seconds)

        def is_shutdown():
            return False


import os
import utm
import overpy
import argparse
import numpy as np
from shapely.ops import linemerge
import shapely.geometry as geometry

try:
    from tqdm import tqdm
except ImportError:

    def tqdm(iter, desc=None, *args, **kwargs):
        if desc is not None:
            rospy.loginfo(desc)
        return iter


from gpxpy import parse as gpxparse

from way import Way


OBSTACLE_RADIUS = 2  # meters, radius of the circle around the obstacle
OSM_RECTANGLE_MARGIN = 100  # meters, margin around the map
RESERVE = 50  # meters, reserve around the waypoints


class CoordsData:
    def __init__(self, min_long, max_long, min_lat, max_lat):
        self.min_long = min_long
        self.max_long = max_long
        self.min_lat = min_lat
        self.max_lat = max_lat
        self.x_margin, self.y_margin = self.get_margin()

    def get_margin(self):
        """
        Get the margin for the map.

        Returns:
        --------
        x_margin : float
            Margin in the x direction.
        y_margin : float
            Margin in the y direction.
        """
        y_margin = (self.max_lat - self.min_lat) * 0.1
        x_margin = (self.max_long - self.min_long) * 0.1

        if y_margin < x_margin:
            y_margin = x_margin
        else:
            x_margin = y_margin

        return x_margin, y_margin


class MapData:
    def __init__(
        self, coords, coords_type="file", current_robot_position=None, flip=False
    ):
        self.api = overpy.Overpass(url="https://overpass.kumi.systems/api/interpreter")

        self.coords_type = coords_type
        if coords_type == "file":
            gpx_f = open(coords, "r")
            gpx_object = gpxparse(gpx_f)
            self.coords_file = coords
            self.waypoints = np.array(
                [[point.latitude, point.longitude] for point in gpx_object.waypoints]
            )
            self.waypoints, self.zone_number, self.zone_letter = self.waypoints_to_utm(
                self.waypoints
            )
        elif coords_type == "array":
            self.waypoints = np.array(coords[0])
            self.zone_number = coords[1]
            self.zone_letter = coords[2]
        elif coords_type == "planner":
            self.waypoints, self.zone_number, self.zone_letter = self.waypoints_to_utm(
                np.array(coords)
            )
        else:
            rospy.logerr("Unknown coords_type.")
            return

        self.flip = flip
        if self.flip:
            self.waypoints = np.flip(self.waypoints, 0)

        if current_robot_position is not None:
            self.robot_position_first_point = True
            self.waypoints = np.concatenate([current_robot_position, self.waypoints])
        else:
            self.robot_position_first_point = False

        self.max_x = np.max(self.waypoints[:, 0]) + RESERVE
        self.min_x = np.min(self.waypoints[:, 0]) - RESERVE
        self.max_y = np.max(self.waypoints[:, 1]) + RESERVE
        self.min_y = np.min(self.waypoints[:, 1]) - RESERVE

        self.max_lat = utm.to_latlon(
            self.max_x + OSM_RECTANGLE_MARGIN,
            self.max_y + OSM_RECTANGLE_MARGIN,
            self.zone_number,
            self.zone_letter,
        )[0]
        self.max_long = utm.to_latlon(
            self.max_x + OSM_RECTANGLE_MARGIN,
            self.max_y + OSM_RECTANGLE_MARGIN,
            self.zone_number,
            self.zone_letter,
        )[1]
        self.min_lat = utm.to_latlon(
            self.min_x - OSM_RECTANGLE_MARGIN,
            self.min_y - OSM_RECTANGLE_MARGIN,
            self.zone_number,
            self.zone_letter,
        )[0]
        self.min_long = utm.to_latlon(
            self.min_x - OSM_RECTANGLE_MARGIN,
            self.min_y - OSM_RECTANGLE_MARGIN,
            self.zone_number,
            self.zone_letter,
        )[1]

        self.coords_data = CoordsData(
            self.min_long, self.max_long, self.min_lat, self.max_lat
        )

        self.points = list(
            map(geometry.Point, zip(self.waypoints[:, 0], self.waypoints[:, 1]))
        )

        self.points_information = []
        self.way_node_ids = set()

        self.osm_ways_data = None
        self.osm_rels_data = None
        self.osm_nodes_data = None

        self.roads = set()
        self.footways = set()
        self.barriers = set()

        self.roads_list = []
        self.footways_list = []
        self.barriers_list = []

        self.ways = dict()

        self._path = os.path.dirname(os.path.dirname(__file__))
        self.BARRIER_TAGS = self.csv_to_dict(
            os.path.join(self._path, "parameters/barrier_tags.csv")
        )
        self.NOT_BARRIER_TAGS = self.csv_to_dict(
            os.path.join(self._path, "parameters/not_barrier_tags.csv")
        )
        self.ANTI_BARRIER_TAGS = self.csv_to_dict(
            os.path.join(self._path, "parameters/anti_barrier_tags.csv")
        )

        self.OBSTACLE_TAGS = self.csv_to_dict(
            os.path.join(self._path, "parameters/barrier_tags.csv")
        )
        self.NOT_OBSTACLE_TAGS = self.csv_to_dict(
            os.path.join(self._path, "parameters/not_obstacle_tags.csv")
        )

    def csv_to_dict(self, f):
        """
        Convert a csv file to a dictionary.

        Parameters:
        -----------
        f : str
            Path to the csv file.

        Returns:
        --------
        dic : dict
            Dictionary of the csv file.
        """
        arr = np.genfromtxt(f, dtype=str, delimiter=",")
        dic = dict()
        for row in arr:
            if row[0] in dic:
                dic[row[0]].append(row[1])
            else:
                dic[row[0]] = [row[1]]
        return dic

    def waypoints_to_utm(self, waypoints):
        """
        Convert waypoints obtained from .gpx file from lat/lon (WGS84) to UTM.

        Parameters:
        -----------
        waypoints : numpy.ndarray
            Array of waypoints in lat/lon format.

        Returns:
        --------
        utm_coords : numpy.ndarray
            Array of waypoints in UTM format.
        zone_number : int
            UTM zone number.
        zone_letter : str
            UTM zone letter.
        """
        utm_arr = utm.from_latlon(waypoints[:, 0], waypoints[:, 1])
        utm_coords = np.concatenate(
            (utm_arr[0].reshape(-1, 1), utm_arr[1].reshape(-1, 1)), axis=1
        )
        zone_number = utm_arr[2]
        zone_letter = utm_arr[3]
        return utm_coords, zone_number, zone_letter

    def get_way_query(self):
        """
        Get query for ways from OSM API.

        Returns:
        --------
        str
            Query for ways.
        """
        return "(way({}, {}, {}, {}); >; ); out;".format(
            self.min_lat, self.min_long, self.max_lat, self.max_long
        )

    def get_rel_query(self):
        """
        Get query for relations from OSM API.

        Returns:
        --------
        str
            Query for relations.
        """
        return "(way({}, {}, {}, {}); <; ); out;".format(
            self.min_lat, self.min_long, self.max_lat, self.max_long
        )

    def get_node_query(self):
        """
        Get query for nodes from OSM API.

        Returns:
        --------
        str
            Query for nodes.
        """
        return "(node({}, {}, {}, {}); ); out;".format(
            self.min_lat, self.min_long, self.max_lat, self.max_long
        )

    def run_queries(self):
        """
        Obtain data from OSM through their API.
        """

        def make_overpy_result_picklable(data):
            def make_overpy_picklable(data):
                if hasattr(data, "_attribute_modifiers"):
                    data._attribute_modifiers = None
                return data

            data = make_overpy_picklable(data)
            for i in range(len(data.nodes)):
                data.nodes[i] = make_overpy_picklable(data.nodes[i])
            for i in range(len(data.ways)):
                data.ways[i] = make_overpy_picklable(data.ways[i])
            for i in range(len(data.areas)):
                data.areas[i] = make_overpy_picklable(data.areas[i])
            for i in range(len(data.relations)):
                data.relations[i] = make_overpy_picklable(data.relations[i])
            return data

        break_time = rospy.Duration(5)
        tries = 1
        while tries < 4 and not rospy.is_shutdown():
            rospy.loginfo("Running 1/3 OSM query.")
            try:
                self.way_query = self.get_way_query()
                osm_ways_data = self.api.query(self.way_query)
                self.osm_ways_data = make_overpy_result_picklable(osm_ways_data)
                break
            except Exception as e:
                rospy.logwarn(e)
                rospy.loginfo(
                    "--------------\nQuery failed.\nRerunning the query after {} s.".format(
                        break_time
                    )
                )
                rospy.sleep(break_time)
                tries += 1

        tries = 1
        while tries < 4 and not rospy.is_shutdown():
            rospy.loginfo("Running 2/3 OSM query.")
            try:
                self.rel_query = self.get_rel_query()
                osm_rels_data = self.api.query(self.rel_query)
                self.osm_rels_data = make_overpy_result_picklable(osm_rels_data)
                break
            except Exception as e:
                rospy.logwarn(e)
                rospy.loginfo(
                    "--------------\nQuery failed.\nRerunning the query after {} s.".format(
                        break_time
                    )
                )
                rospy.sleep(break_time)
                tries += 1

        tries = 1
        while tries < 4 and not rospy.is_shutdown():
            rospy.loginfo("Running 3/3 OSM query.")
            try:
                self.node_query = self.get_node_query()
                osm_nodes_data = self.api.query(self.node_query)
                self.osm_nodes_data = make_overpy_result_picklable(osm_nodes_data)
                break
            except Exception as e:
                rospy.logwarn(e)
                rospy.loginfo(
                    "--------------\nQuery failed.\nRerunning the query after {} s.".format(
                        break_time
                    )
                )
                rospy.sleep(break_time)
                tries += 1

        rospy.loginfo("Queries finished.")

    def combine_ways(self, ids):
        """
        Combine ways that share a node.

        Parameters:
        -----------
        ids : list
            List of way ids.

        Returns:
        --------
        ids : list
            List of combined way ids.
        """
        ways = []
        for id in ids:
            ways.append(self.ways[id])
        i = 0
        while i < len(ways):
            j = 0
            while j < len(ways):
                if i != j and (not ways[i].is_area and not ways[j].is_area):
                    if ways[i].nodes[0].id == ways[j].nodes[0].id:
                        ways[i].nodes.reverse()
                    elif ways[i].nodes[-1].id == ways[j].nodes[-1].id:
                        ways[j].nodes.reverse()

                    if ways[i].nodes[-1].id == ways[j].nodes[0].id:
                        combined_line = linemerge([ways[i].line, ways[j].line])

                        new_way = Way()
                        new_way.id = int(-(10**15) * np.random.random())
                        while new_way.id in self.ways.keys():
                            new_way.id = int(-(10**15) * np.random.random())
                        # TODO: tady zlobi ten update, podivat se na prirazovani id
                        new_way.nodes = ways[i].nodes + ways[j].nodes[1:]

                        if ways[i].tags is None:
                            ways[i].tags = dict()
                        if ways[j].tags is None:
                            ways[j].tags = dict()
                        new_way.tags = ways[i].tags.update(ways[j].tags)
                        new_way.line = combined_line

                        if new_way.nodes[0].id == new_way.nodes[-1].id:
                            new_way.is_area = True
                            new_way.line = geometry.Polygon(new_way.line.coords)
                        self.ways[new_way.id] = new_way
                        ways[j] = new_way
                        ids[j] = new_way.id
                        ids.pop(i)
                        ways.pop(i)
                        i -= 1
                        j -= 1
                        break
                j += 1
            i += 1

        return ids

    def parse_ways(self):
        """
        Fill self.ways, a dictionary of id:way pairs, from all the ways from the query.
        """
        for way in tqdm(self.osm_ways_data.ways, desc="Parse ways"):
            way_to_store = Way()
            coords = []

            # Convert WGS -> UTM.
            lats = np.array([float(node.lat) for node in way.nodes])
            lons = np.array([float(node.lon) for node in way.nodes])
            utm_coords = utm.from_latlon(lats, lons)
            coords = list(zip(utm_coords[0], utm_coords[1]))

            # Keep track of IDs of each node, so that in parse_nodes we can distinguish them from solitary nodes.
            ids = [node.id for node in way.nodes]
            if self.way_node_ids is None:
                self.way_node_ids = dict()
            self.way_node_ids.update(ids)

            # Distinguish areas and non-areas (we use a single class for both cases).
            if coords[0] == coords[-1]:
                way_to_store.is_area = True
            else:
                way_to_store.is_area = False

            way_to_store.id = way.id
            way_to_store.nodes = way.nodes
            way_to_store.tags = way.tags

            if way_to_store.tags is None:
                way_to_store.tags = dict()

            if way_to_store.is_area:
                way_to_store.line = geometry.Polygon(coords)
            else:
                way_to_store.line = geometry.LineString(coords)

            self.ways[way.id] = way_to_store

    def parse_rels(self):
        """
        Needs self.ways DICTIONARY (key is id) with a self.is_area parameter, which is obtained from parse_ways.
        Use relations to alter ways - combine neighbor ways, add tags...
        """
        for rel in tqdm(self.osm_rels_data.relations, desc="Parse rels"):
            inner_ids = []
            outer_ids = []
            keys = self.ways.keys()

            # Separate inner and outer ways of relation.
            for member in rel.members:
                if member._type_value == "way":
                    if int(member.ref) in keys:
                        if member.role == "outer":
                            outer_ids.append(int(member.ref))
                        else:
                            inner_ids.append(int(member.ref))

            # If two ways are "connected" (they share a node), combine them into one.
            outer_ids = self.combine_ways(outer_ids)

            for id in outer_ids:
                way = self.ways[id]
                way.in_out = "outer"

                if way.tags is None:
                    way.tags = dict()
                if rel.tags is None:
                    rel.tags = dict()
                way.tags.update(rel.tags)

                self.ways[id] = way

            for id in inner_ids:
                way = self.ways[id]
                way.in_out = "inner"
                self.ways[id] = way

    def parse_nodes(self):
        """
        Convert solitary nodes (not part of a way) to barrier areas.
        """
        for node in tqdm(self.osm_nodes_data.nodes, desc="Parse nodes"):
            if node.id not in self.way_node_ids:
                # Check if node is a obstacle.
                if any(
                    key in self.OBSTACLE_TAGS
                    and (
                        node.tags[key] in self.OBSTACLE_TAGS[key]
                        or (
                            "*" in self.OBSTACLE_TAGS[key]
                            and node.tags[key]
                            not in self.NOT_OBSTACLE_TAGS.get(key, [])
                        )
                    )
                    for key in node.tags
                ):
                    obstacle = Way()
                    obstacle.id = node.id
                    obstacle.is_area = True
                    obstacle.tags = node.tags

                    coords = utm.from_latlon(float(node.lat), float(node.lon))
                    polygon = self.point_to_polygon(
                        geometry.Point([coords[0], coords[1]]), OBSTACLE_RADIUS
                    )
                    obstacle.line = polygon

                    self.barriers.add(obstacle)

    def get_points(self, z=0):
        """
        Return all nodes as dictionary of id:utm coords.

        Parameters:
        -----------
        z : float
            Altitude of the points.

        Returns:
        --------
        points : dict
            Dictionary of id:utm coords.
        """
        points = {}
        for node in self.osm_nodes_data.nodes:
            id, lat, lon = node.id, float(node.lat), float(node.lon)
            easting, northing, _, _ = utm.from_latlon(lat, lon)
            points[id] = np.array([easting, northing, z]).reshape(3, 1)
        for way in self.osm_ways_data.ways:
            for node in way.nodes:
                id, lat, lon = node.id, float(node.lat), float(node.lon)
                easting, northing, _, _ = utm.from_latlon(lat, lon)
                points[id] = np.array([easting, northing, z]).reshape(3, 1)
        return points

    def get_ways(self):
        """
        Return all ways as dictionary of type:list.

        Returns:
        --------
        ways : dict
            Dictionary of type:list.
        """
        return {
            "roads": self.roads_list,
            "footways": self.footways_list,
            "barriers": self.barriers_list,
        }

    def point_to_polygon(self, point, r=1):
        """
        Convert a node (= a point) to a circle area, with a given radius in meters.

        Parameters:
        -----------
        point : shapely.geometry.Point
            Point to convert.
        r : float
            Radius of the circle.
        """
        return point.buffer(r)

    def line_to_polygon(self, way, width=4):
        """
        The width of the buffer should depend on, the type of way (river x fence, highway x path)...

        Parameters:
        -----------
        way : Way
            Way to convert.
        width : float
            Width of the buffer.
        """
        way.line = way.line.buffer(width / 2)
        way.is_area = True
        return way

    def separate_ways(self):
        """
        Separate ways (dict) into roads, footways and barriers (lists).
        """
        for way in tqdm(self.ways.values(), desc="Separate ways"):
            if way.is_road():
                way = self.line_to_polygon(way, width=7)
                self.roads.add(way)

            elif way.is_footway():
                way = self.line_to_polygon(way, width=3)
                self.footways.add(way)

            elif way.is_barrier(
                self.BARRIER_TAGS, self.NOT_BARRIER_TAGS, self.ANTI_BARRIER_TAGS
            ):
                if not way.is_area:
                    way = self.line_to_polygon(way, width=2)
                self.barriers.add(way)

    def sets_to_lists(self):
        """
        Convert sets of parsed osm ways into lists.
        """
        self.roads_list = list(self.roads)
        self.footways_list = list(self.footways)
        self.barriers_list = list(self.barriers)

    def run_parse(self):
        """
        Parse OSM data into their respective categories.

        Returns:
        --------
        int
            0 if successful, 1 if failed.
        """
        end = False
        rospy.loginfo("Running analysis.")
        if self.osm_ways_data is not None:
            self.parse_ways()
            rospy.loginfo("Ways parsed.")
        else:
            rospy.logwarn("No ways data.")
            end = True
        if self.osm_rels_data is not None:
            self.parse_rels()
            rospy.loginfo("Rels parsed.")
        else:
            rospy.logwarn("No rels data.")
            end = True
        if self.osm_nodes_data is not None:
            self.parse_nodes()
            rospy.loginfo("Nodes parsed.")
        else:
            rospy.logwarn("No nodes data.")
            end = True

        if end:
            rospy.logerr("Analysis failed.")
            return 1

        self.separate_ways()
        self.sets_to_lists()
        rospy.loginfo("Analysis finished.")
        return 0

    def run_all(self, save=True):
        """
        Run all queries and parsing in one go.

        Parameters:
        -----------
        save : bool
            Whether to save the data to a pickle file.
        """
        self.run_queries()
        if self.run_parse():
            return
        if save:
            self.save_to_pickle()

    def save_to_pickle(self, filename=None):
        """
        Save map data to a pickle file.

        Parameters:
        -----------
        filename : str
            Path and name to the file.
        """
        fn = self.coords_file if self.coords_type == "file" else filename
        if fn is None:
            rospy.logerr("No filename given.")
            return
        try:
            with open(fn[:-4] + ".mapdata", "wb") as fh:
                pickle.dump(self, fh, protocol=2)
            rospy.loginfo("Map data saved to {}".format(fn[:-4] + ".mapdata"))
        except Exception as e:
            rospy.logerr("Error while saving map data: {}".format(e))


def parse_args():
    parser = argparse.ArgumentParser(description="Map data parser for OSM.")

    parser.add_argument("-d", action="store_true", help="Download data from OSM.")
    parser.add_argument(
        "-f",
        type=str,
        default="buchlovice.gpx",
        help="File name of the .gpx file with waypoints.",
    )

    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    if args.d:
        map_data = MapData(
            os.path.join(os.path.dirname(__file__), "../data", args.f),
            coords_type="file",
        )
        map_data.run_queries()
    else:
        with open(
            os.path.join(
                os.path.dirname(__file__), "../data", args.f[:-4] + ".mapdata"
            ),
            "rb",
        ) as fh:
            map_data = pickle.load(fh)

    if map_data.run_parse():
        exit(1)
    map_data.save_to_pickle()
