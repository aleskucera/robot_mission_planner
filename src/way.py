import numpy as np
from shapely.geometry import Point, MultiPoint, LineString
from shapely.prepared import prep

from map_data.points_to_graph_points import get_point_line


# living_street, pedestrian, track, crossing can be accessed by cars
FOOTWAY_VALUES = ['living_street', 'pedestrian', 'footway', 'bridleway', 'corridor', 'track', 'steps', 'cycleway', 'path']


class Way():
    def __init__(self, id=-1, is_area=False, nodes=[], tags=None, line=None, in_out=""):
        self.id = id
        self.is_area = is_area
        self.nodes = nodes
        self.tags = tags
        self.line = line
        self.in_out = in_out

        self.pcd_points = None
    
    def is_road(self):
        '''
        Check if the way is a road.

        Returns:
        --------
        bool
            Whether the way is a road or not.
        '''
        if self.tags.get('highway', None) and not self.tags.get('highway', None) in FOOTWAY_VALUES:
            return True

    def is_footway(self):
        '''
        Check if the way is a footway.

        Returns:
        --------
        bool
            Whether the way is a footway or not.
        '''
        if self.tags.get('highway', None) and self.tags.get('highway', None) in FOOTWAY_VALUES:
            return True
        
    def is_barrier(self, yes_tags, not_tags, anti_tags):
        '''
        Check if the way is a barrier.

        Parameters:
        -----------
        yes_tags : dict
            Dictionary of tags that signify barrier.
        not_tags : dict
            Dictionary of tags that signify not barrier.
        anti_tags : dict
            Dictionary of tags that signify automaticaly disqualify from being a barrier.

        Returns:
        --------
        bool
            Whether the way is a barrier or not.
        '''
        if any(key in yes_tags and (self.tags[key] in yes_tags[key] or ('*' in yes_tags[key] and not self.tags[key] in not_tags.get(key,[])))\
               for key in self.tags) and not any(key in anti_tags and (self.tags[key] in anti_tags[key]) for key in self.tags):
            return True

    def to_pcd_points(self, density=2, filled=True):
        '''
        Create a point cloud from the way using a meshgrid.

        Parameters:
        -----------
        density : int
            Density of the meshgrid.
        filled : bool
            Whether the point cloud should be filled or not.

        Returns:
        --------
        pcd_points : np.array
            Point cloud of the way.
        '''
        # https://stackoverflow.com/questions/44399749/get-all-lattice-points-lying-inside-a-shapely-polygon
        
        if self.pcd_points is None:
            if filled:
                xmin, ymin, xmax, ymax = self.line.bounds
                x = np.arange(np.floor(xmin * density) / density, np.ceil(xmax * density) / density, 1 / density) 
                y = np.arange(np.floor(ymin * density) / density, np.ceil(ymax * density) / density, 1 / density)
                xv, yv = np.meshgrid(x, y)
                xv = xv.ravel()
                yv = yv.ravel()
                points = MultiPoint(np.array([xv,yv]).T).geoms
        
                points = self.mask_points(points,self.line)
                self.pcd_points = list(points)
                self.pcd_points = np.array(list(LineString(self.pcd_points).xy)).T
            else:
                points = self.line.exterior.coords
                pcd_points = np.array([]).reshape((0,2))
                
                for i in range(len(points)):
                    if i+1 <= len(points)-1:
                        p1 = Point(points[i])
                        p2 = Point(points[i+1])
                    else:
                        p1 = Point(points[i])
                        p2 = Point(points[0])

                    _, line, _ = get_point_line(p1,p2,0.5)
                    pcd_points = np.concatenate([pcd_points,line])
                self.pcd_points = pcd_points
        
        return self.pcd_points
    
    def mask_points(self, points, polygon):
        '''
        Mask points with a polygon.

        Parameters:
        -----------
        points : shapely.geometry.MultiPoint
            List of points.
        polygon : shapely.geometry.Polygon
            Polygon to mask the points with.

        Returns:
        --------
        list
            Masked points.
        '''
        polygon = prep(polygon)
        contains = lambda p: polygon.contains(p)

        return filter(contains, points)
