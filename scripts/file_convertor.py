import os
import sys
import argparse

import yaml

sys.path.append(
    f"{os.path.dirname(__file__)}/../src"
)  # Adjust the path to your project structure
from utils import create_gpx_content


def convert_yaml_to_gpx(yaml_file_path, gpx_file_path):
    """
    Reads a YAML file, extracts waypoints, and writes them to a GPX file.
    """
    print(f"Reading waypoints from: {yaml_file_path}")

    try:
        with open(yaml_file_path, "r") as file:
            data = yaml.safe_load(file)
    except FileNotFoundError:
        print(f"Error: Input file not found at '{yaml_file_path}'", file=sys.stderr)
        sys.exit(1)
    except yaml.YAMLError as e:
        print(f"Error: Could not parse YAML file. Details: {e}", file=sys.stderr)
        sys.exit(1)

    # Check if the 'waypoints' key exists and is a list
    if "waypoints" not in data or not isinstance(data["waypoints"], list):
        print(
            "Error: YAML file must contain a top-level key 'waypoints' with a list of points.",
            file=sys.stderr,
        )
        sys.exit(1)

    waypoints = data["waypoints"]
    if not waypoints:
        print(
            "Warning: The 'waypoints' list in the YAML file is empty. An empty GPX file will be created.",
            file=sys.stderr,
        )

    # Generate the GPX XML content
    gpx_content = create_gpx_content(waypoints)

    print(f"Writing {len(waypoints)} waypoints to: {gpx_file_path}")
    try:
        with open(gpx_file_path, "w") as file:
            file.write(gpx_content)
    except IOError as e:
        print(
            f"Error: Could not write to output file '{gpx_file_path}'. Details: {e}",
            file=sys.stderr,
        )
        sys.exit(1)

    print("Conversion successful!")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert a YAML file containing geographic waypoints to a GPX file."
    )
    parser.add_argument(
        "--input",
        required=True,
        help="Path to the input YAML file (e.g., wc_anlage025.yaml)",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Path to the output GPX file (e.g., output.gpx). If not provided, it will be based on the input filename.",
    )

    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    if not args.input.endswith((".yaml", ".yml")):
        print(
            "Error: Input file must be a YAML or GPX file with.",
            file=sys.stderr,
        )
        sys.exit(1)

    # Which conversion to perform
    yaml_2_gpx = args.input.lower().endswith((".yaml", ".yml"))

    # If output filename is not provided, create one from the input filename
    output_filename = args.output
    if output_filename is None:
        # Replaces .yaml (or .yml) with .gpx
        base_name = args.input.rsplit(".", 1)[0]
        if yaml_2_gpx:
            output_filename = f"{base_name}.gpx"
        else:
            output_filename = f"{base_name}.yaml"

    if yaml_2_gpx:
        convert_yaml_to_gpx(args.input, output_filename)
    else:
        convert_gpx_to_yaml(args.input, output_filename)
