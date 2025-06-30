from flask import Flask
from flask import render_template

from src.wormhole.manager import wormhole_manager_instance


def create_app():
    """Application factory."""
    app = Flask(__name__, static_folder="static", template_folder="templates")

    # Initialize the wormhole manager with the app instance
    wormhole_manager_instance.init_app(app)

    # Register the main API blueprint
    from src.api.routes import api_bp

    app.register_blueprint(api_bp, url_prefix="/api")

    # --- ADD THIS ---
    # Register the new DEM tile server blueprint
    from src.api.tile_routes import tiles_bp

    app.register_blueprint(tiles_bp, url_prefix="/tiles")
    # --- END ADD ---

    # A simple route for the main page
    @app.route("/")
    def index():
        return render_template("index.html")

    return app


app = create_app()

if __name__ == "__main__":
    app.run(debug=True, use_reloader=False)
