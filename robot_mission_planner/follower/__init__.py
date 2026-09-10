"""Building blocks of the follower node (``road_follower``).

The node itself is a builder and a 1 Hz tick; everything it is made of lives here:
frames and transforms (``frames``), the route it follows (``route``), how the next road
goal is chosen (``road_goal``), and the navigation backend it talks to (``backends``).
"""
