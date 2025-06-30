import os

from flask import Blueprint
from flask import current_app
from flask import send_from_directory

# This blueprint now serves pre-computed static tiles
tiles_bp = Blueprint("tiles", __name__)

# The directory where your precomputed tiles are stored.
# We build an absolute path from the application's root.
PRECOMPUTED_DIR_NAME = "precomputed_tiles"


@tiles_bp.route("/dem/<int:z>/<int:x>/<int:y>.png")
def dem_tile_server(z, x, y):
    """
    Serves a pre-computed DEM map tile from the static directory.
    This is extremely fast as it involves no real-time computation.
    """
    # Construct the absolute path to the precomputed_tiles directory
    tiles_dir = os.path.join(current_app.root_path, PRECOMPUTED_DIR_NAME)

    # We need to serve from the sub-directory for the zoom level
    tile_subdir = os.path.join(str(z), str(x))

    # Use Flask's send_from_directory which is secure and efficient
    return send_from_directory(os.path.join(tiles_dir, tile_subdir), f"{y}.png")
