import os

import matplotlib.cm as cm
import mercantile
import numpy as np
import rasterio
from PIL import Image
from rasterio.warp import transform_bounds
from rasterio.windows import from_bounds
from tqdm import tqdm

# --- CONFIGURATION ---
SOURCE_GEOTIFF = "data/ogd-10m-at/dhm_at_lamb_10m_2018.tif"
OUTPUT_DIR = "precomputed_tiles"  # Tiles will be saved here
MIN_ZOOM = 7
MAX_ZOOM = 15  # Be careful: z16 would be 4x more tiles than z15. Start with 15.

# Elevation range for Austria in meters for consistent coloring
VMIN, VMAX = 100, 3800


def precompute_tiles():
    """
    Iterates through map tiles for specified zoom levels, renders them from the
    GeoTIFF, and saves them as PNG files.
    """
    if not os.path.exists(SOURCE_GEOTIFF):
        print(f"ERROR: Source GeoTIFF not found at '{SOURCE_GEOTIFF}'")
        return

    print(f"Starting tile pre-computation from zoom {MIN_ZOOM} to {MAX_ZOOM}.")
    print(f"Output directory: {OUTPUT_DIR}")

    with rasterio.open(SOURCE_GEOTIFF) as src:
        # Get the total bounding box of the GeoTIFF in lat/lon
        total_bounds_native = src.bounds
        total_bounds_latlon = transform_bounds(
            src.crs, "EPSG:4326", *total_bounds_native
        )

        for z in range(MIN_ZOOM, MAX_ZOOM + 1):
            print(f"\nProcessing Zoom Level: {z}")

            # Get all tile coordinates that cover our GeoTIFF's bounding box
            tiles = list(mercantile.tiles(*total_bounds_latlon, zooms=[z]))

            # Use tqdm for a progress bar
            for tile in tqdm(tiles, desc=f"Zoom {z}"):
                try:
                    # Get the geographic bounds (lat/lon) of this specific tile
                    latlon_bounds = mercantile.bounds(tile)

                    # Transform these bounds to the GeoTIFF's native coordinate system
                    native_bounds = transform_bounds(
                        "EPSG:4326", src.crs, *latlon_bounds
                    )

                    # Calculate the pixel window to read
                    window = from_bounds(*native_bounds, src.transform)

                    # Read only that window, resampling to a 256x256 tile
                    data = src.read(
                        1,
                        window=window,
                        out_shape=(256, 256),
                        resampling=rasterio.enums.Resampling.bilinear,
                    )

                    # --- This is the same coloring logic as before ---
                    nodata_val = src.nodata
                    if nodata_val is not None:
                        data[data == nodata_val] = np.nan

                    # Skip empty tiles
                    if np.all(np.isnan(data)):
                        continue

                    norm = np.clip((data - VMIN) / (VMAX - VMIN), 0, 1)
                    rgba_data = cm.terrain(norm, bytes=True)
                    rgba_data[np.isnan(norm)] = [0, 0, 0, 0]

                    # --- Save the tile to a file ---
                    tile_dir = os.path.join(OUTPUT_DIR, str(z), str(tile.x))
                    os.makedirs(tile_dir, exist_ok=True)

                    tile_path = os.path.join(tile_dir, f"{tile.y}.png")

                    img = Image.fromarray(rgba_data, "RGBA")
                    img.save(tile_path, "PNG")

                except Exception as e:
                    # This might happen for tiles that barely touch the edge; it's safe to ignore
                    # print(f"Warning: Could not process tile {z}/{tile.x}/{tile.y}. Reason: {e}")
                    pass

    print("\nTile pre-computation finished successfully!")


if __name__ == "__main__":
    precompute_tiles()
