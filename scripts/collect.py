#!/usr/bin/env python3

import rasterio
import numpy as np
import matplotlib.pyplot as plt
from rasterio.windows import Window
from rasterio.enums import Resampling

# Use the path to your downloaded TIFF file
filepath = "./data/ogd-10m-at/dhm_at_lamb_10m_2018.tif"

# --- Strategy 1: Visualize a Downsampled Overview ---
# We will read the data at a lower resolution to fit it in memory.

# Define a downsampling factor. A factor of 50 means we read 1 pixel for every 50x50 block.
downsample_factor = 50

try:
    with rasterio.open(filepath) as src:
        print(f"Successfully opened: {filepath}")
        print(f"Coordinate System: {src.crs}")
        print(f"Original Dimensions: {src.width} x {src.height}")

        # Calculate the new, downsampled shape
        new_width = src.width // downsample_factor
        new_height = src.height // downsample_factor

        # Read the data, downsampling it on the fly.
        # This is the key to handling large files efficiently.
        # `Resampling.bilinear` averages pixels, which is good for elevation data.
        overview_data = src.read(
            1,  # Read the first band
            out_shape=(new_height, new_width),
            resampling=Resampling.bilinear,
        )

        print(f"Downsampled to: {overview_data.shape}")

        # Handle NoData values: raster data often uses a specific large negative
        # number for areas with no data. We replace it with NaN for plotting.
        nodata_val = src.nodata
        if nodata_val is not None:
            overview_data[overview_data == nodata_val] = np.nan

except FileNotFoundError:
    print(f"ERROR: The file '{filepath}' was not found.")
    print("Please make sure the script is in the same directory as your .tif file.")
    overview_data = None


# Plot the downsampled overview
if overview_data is not None:
    plt.figure(figsize=(12, 8))
    plt.imshow(overview_data, cmap="terrain")
    plt.colorbar(label="Elevation (meters)")
    plt.title(f"Overview of Austrian 10m DEM (Downsampled 1:{downsample_factor})")
    plt.xlabel("Pixel Column (Downsampled)")
    plt.ylabel("Pixel Row (Downsampled)")
    plt.show()


# --- Strategy 2: Visualize a Full-Resolution Detail Window ---
# Now, let's read a small 2000x2000 pixel window (20km x 20km) at full resolution
# from the center of the dataset to see the detail.

try:
    with rasterio.open(filepath) as src:
        # Define a window in the middle of the raster
        window_size = 2000
        col_offset = (src.width - window_size) // 2
        row_offset = (src.height - window_size) // 2

        detail_window = Window(col_offset, row_offset, window_size, window_size)

        print(f"\nReading a {window_size}x{window_size} window at full resolution...")
        detail_data = src.read(1, window=detail_window)

        # Handle NoData values for the detail window
        nodata_val = src.nodata
        if nodata_val is not None:
            detail_data[detail_data == nodata_val] = np.nan

except (FileNotFoundError, NameError):
    # This will skip if the file wasn't found in the first place
    detail_data = None

# Plot the full-resolution detail window
if detail_data is not None:
    plt.figure(figsize=(10, 10))
    plt.imshow(detail_data, cmap="terrain")
    plt.colorbar(label="Elevation (meters)")
    plt.title(f"Full Resolution Detail (20km x 20km)")
    plt.xlabel("Pixel Column")
    plt.ylabel("Pixel Row")
    plt.show()
