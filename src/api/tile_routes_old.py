import io
import os

import matplotlib.cm as cm
import mercantile
import numpy as np
import rasterio
from flask import Blueprint
from flask import current_app
from flask import Response
from PIL import Image
from rasterio.warp import transform_bounds
from rasterio.windows import from_bounds

# Create a new Blueprint for our tile server
tiles_bp = Blueprint("tiles", __name__)

# Define the path to the GeoTIFF relative to the project root
# This is more robust than a simple relative path.
GEOTIFF_FILENAME = "data/ogd-10m-at/dhm_at_lamb_10m_2018.tif"

# We'll cache the file path to avoid recalculating it on every tile request
geotiff_path_cache = None


def get_geotiff_path():
    """Finds and caches the absolute path to the GeoTIFF."""
    global geotiff_path_cache
    if geotiff_path_cache is None:
        # current_app.root_path is the path to the project's root folder ('robot_mission_planner')
        path = os.path.join(current_app.root_path, GEOTIFF_FILENAME)
        if not os.path.exists(path):
            current_app.logger.error(f"GeoTIFF file not found at expected path: {path}")
            return None
        geotiff_path_cache = path
    return geotiff_path_cache


@tiles_bp.route("/dem/<int:z>/<int:x>/<int:y>.png")
def dem_tile_server(z, x, y):
    """
    Generates and serves a DEM map tile on-the-fly.
    """
    filepath = get_geotiff_path()
    if not filepath:
        # Return a 404 if the source file isn't found
        return "DEM source file not configured or found on server", 404

    try:
        with rasterio.open(filepath) as src:
            # Get the geographic bounds (lat/lon) of the requested tile
            latlon_bounds = mercantile.bounds(x, y, z)

            # Convert these bounds to the GeoTIFF's native coordinate system
            native_bounds = transform_bounds("EPSG:4326", src.crs, *latlon_bounds)

            # Calculate the pixel window in the GeoTIFF for these bounds
            window = from_bounds(*native_bounds, src.transform)

            # Read only that window of data, outputting a 256x256 tile
            data = src.read(
                1,
                window=window,
                out_shape=(256, 256),
                resampling=rasterio.enums.Resampling.bilinear,
            )

            # Colorize the data
            nodata_val = src.nodata
            if nodata_val is not None:
                data[data == nodata_val] = np.nan

            vmin, vmax = 100, 3800  # Elevation range for Austria in meters
            norm = np.clip((data - vmin) / (vmax - vmin), 0, 1)

            if np.all(np.isnan(norm)):
                return Response(
                    Image.new("RGBA", (256, 256), (0, 0, 0, 0)).tobytes(),
                    mimetype="image/png",
                )

            rgba_data = cm.terrain(norm, bytes=True)
            rgba_data[np.isnan(norm)] = [0, 0, 0, 0]  # Make no-data pixels transparent

            img = Image.fromarray(rgba_data, "RGBA")
            img_byte_arr = io.BytesIO()
            img.save(img_byte_arr, format="PNG")
            img_byte_arr.seek(0)

            return Response(img_byte_arr, mimetype="image/png")

    except Exception as e:
        current_app.logger.error(
            f"Error generating tile {z}/{x}/{y}: {e}", exc_info=True
        )
        # Return a transparent tile on error
        return Response(
            Image.new("RGBA", (256, 256), (0, 0, 0, 0)).tobytes(), mimetype="image/png"
        )
