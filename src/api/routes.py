from flask import Blueprint
from flask import jsonify
from flask import request

from src.path_solver.solver import solver_instance
from src.wormhole.manager import wormhole_manager_instance

api_bp = Blueprint("api", __name__)


@api_bp.route("/add_point", methods=["POST"])
def add_point():
    data = request.json
    point_id = solver_instance.add_point(data["lat"], data["lng"], data["type"])
    return jsonify(
        {
            "success": True,
            "point_id": point_id,
            "total_points": len(solver_instance.points),
        }
    )


@api_bp.route("/solve_path", methods=["POST"])
def solve_path():
    path, message = solver_instance.solve_path()
    if path is None:
        return jsonify({"success": False, "message": message})

    path_coords = [
        {"lat": p["lat"], "lng": p["lng"], "type": p["type"], "id": p["id"]}
        for p in [solver_instance.points[i] for i in path]
    ]
    return jsonify({"success": True, "path": path_coords, "message": message})


@api_bp.route("/clear_points", methods=["POST"])
def clear_points():
    solver_instance.clear_points()
    return jsonify({"success": True})


@api_bp.route("/create_wormhole", methods=["POST"])
def create_wormhole():
    gpx_data = request.json.get("gpx")
    if not gpx_data:
        return jsonify({"success": False, "message": "No GPX data provided"}), 400

    try:
        # Start the transfer process
        transfer_id = wormhole_manager_instance.create_transfer(gpx_data)

        # Wait for the code
        code = wormhole_manager_instance.get_transfer_code(
            transfer_id, timeout=15
        )  # Increased timeout slightly

        if code:
            return jsonify({"success": True, "code": code, "transfer_id": transfer_id})
        else:
            # If we time out, log it and cancel the transfer
            logger = wormhole_manager_instance.app.logger
            logger.error(f"Failed to get wormhole code for transfer {transfer_id}")
            wormhole_manager_instance.cancel_transfer(transfer_id)  # Ensure cleanup
            return (
                jsonify(
                    {
                        "success": False,
                        "message": "Failed to capture wormhole code in time",
                    }
                ),
                500,
            )

    except Exception as e:
        logger = wormhole_manager_instance.app.logger
        logger.error(f"Error creating wormhole: {e}", exc_info=True)
        return jsonify({"success": False, "message": str(e)}), 500


@api_bp.route("/cancel_wormhole", methods=["POST"])
def cancel_wormhole():
    transfer_id = request.json.get("transfer_id")
    success, message = wormhole_manager_instance.cancel_transfer(transfer_id)
    return jsonify({"success": success, "message": message})
