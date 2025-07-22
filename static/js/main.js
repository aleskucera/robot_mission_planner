import { ApiService } from "./modules/ApiService.js";
import { UIManager } from "./modules/UIManager.js";
import { MapManager } from "./modules/MapManager.js";

class PathSolverApp {
  constructor() {
    this.state = {
      markers: [],
      pointCounts: { start: 0, goal: 0, intermediate: 0 },
      pathCoords: [],
      currentMode: null,
      isProcessing: false,
      isDragging: false,
      lastDragEndTime: 0,
      activeTransferId: null,
    };
    this.ui = new UIManager();
    this.map = new MapManager("map");
    this.api = ApiService;
    this.init();
  }

  init() {
    this.bindEventListeners();
    this.updateUI();
  }

  bindEventListeners() {
    document.querySelectorAll(".point-btn").forEach((btn) => {
      btn.addEventListener("click", () =>
        this.selectPointMode(btn.dataset.type),
      );
    });
    document
      .getElementById("solve-btn")
      .addEventListener("click", () => this.solvePath());
    document
      .getElementById("clear-btn")
      .addEventListener("click", () => this.clearAll());
    document
      .getElementById("export-gpx-btn")
      .addEventListener("click", () => this.exportPathToGPX());
    document
      .getElementById("export-wormhole-btn")
      .addEventListener("click", () => this.sharePathViaWormhole());

    document
      .getElementById("gpx-input")
      .addEventListener("change", (e) => this.handleGpxImport(e));

    this.map.on("mapClick", (e) => this.handleMapClick(e.latlng));
    this.map.on("markerDragEnd", (data) => this.handleMarkerDrag(data));
    this.map.on("markerRightClick", (data) =>
      this.handleMarkerRightClick(data),
    );
    this.map.on("gpxPointsLoaded", (data) => this.handleGpxPoints(data.coords));
    this.map.on("gpxLoadError", (data) =>
      this.ui.showStatus(`Error loading GPX: ${data.error}`, "error"),
    );

    document.getElementById("export-btn").addEventListener("click", (e) => {
      e.stopPropagation();
      document.querySelector(".export-options").classList.toggle("show");
    });
    document.addEventListener("click", () =>
      document.querySelector(".export-options").classList.remove("show"),
    );
  }

  updateUI() {
    this.ui.updatePointCount(this.state.pointCounts);
    this.ui.setSolveButtonState(
      this.state.pointCounts.start > 0 && this.state.pointCounts.goal > 0,
    );
    this.ui.setExportButtonState(this.state.pathCoords.length > 0);
  }

  setProcessing(isProcessing) {
    this.state.isProcessing = isProcessing;
    this.ui.setProcessingState(isProcessing, this.state.pointCounts);
  }

  selectPointMode(mode) {
    this.state.currentMode = mode;
    this.ui.selectPointMode(mode);
    this.ui.showStatus(`Click on map to place a ${mode} point.`, "info");
  }

  handleGpxImport(event) {
    const file = event.target.files[0];
    if (!file) return;

    this.ui.showStatus(`Importing ${file.name}...`, "info");
    const reader = new FileReader();
    reader.onload = (e) => {
      this.map.displayGpxTrack(e.target.result);
    };
    reader.onerror = () => {
      this.ui.showStatus(`Error reading file: ${reader.error}`, "error");
    };
    reader.readAsText(file);
    event.target.value = "";
  }

  async handleMapClick(latlng) {
    if (
      !this.state.currentMode ||
      this.state.isProcessing ||
      this.state.isDragging ||
      Date.now() - this.state.lastDragEndTime < 200
    )
      return;

    if (
      this.state.currentMode !== "intermediate" &&
      this.state.pointCounts[this.state.currentMode] > 0
    ) {
      await this.clearPointsByType(this.state.currentMode);
    }
    this.addPoint(latlng.lat, latlng.lng, this.state.currentMode);
  }

  async handleMarkerDrag({ marker }) {
    this.state.isDragging = false;
    this.state.lastDragEndTime = Date.now();
    this.ui.showStatus(`Syncing moved point...`, "info", 1500);
    await this.syncAllPointsToServer();
    if (this.state.pathCoords.length > 0) {
      this.solvePath();
    }
  }

  handleMarkerRightClick({ marker, latlng }) {
    L.DomEvent.preventDefault(event);
    const content = this.ui.showContextMenu({
      latlng,
      ondelete: () => {
        this.map.closeAllPopups();
        this.deleteMarker(marker);
      },
      oncopy: async () => {
        this.map.closeAllPopups();
        const coords = `${marker.getLatLng().lat.toFixed(6)}, ${marker.getLatLng().lng.toFixed(6)}`;
        await navigator.clipboard.writeText(coords);
        this.ui.showStatus("Coordinates copied to clipboard.", "success");
      },
    });
    this.map.showPopup(latlng, content);
  }

  async addPoint(lat, lng, type) {
    try {
      const serverResponse = await this.api.addPoint(lat, lng, type);
      if (serverResponse.success) {
        const leafletMarker = this.map.createMarker(lat, lng, type);
        this.state.markers.push({
          id: serverResponse.point_id,
          type,
          leafletMarker,
        });
        this.state.pointCounts[type]++;
        this.ui.showStatus(
          `${type.charAt(0).toUpperCase() + type.slice(1)} point added.`,
          "success",
        );
        this.updateUI();
      }
    } catch (error) {
      this.ui.showStatus(`Error adding point: ${error.message}`, "error");
    }
  }

  async deleteMarker(markerToDelete) {
    const markerIndex = this.state.markers.findIndex(
      (m) => m.leafletMarker === markerToDelete,
    );
    if (markerIndex === -1) return;
    const { type } = this.state.markers[markerIndex];
    this.map.removeMarker(markerToDelete);
    this.state.markers.splice(markerIndex, 1);
    this.state.pointCounts[type]--;
    if (
      (type === "start" || type === "goal") &&
      this.state.pathCoords.length > 0
    ) {
      this.map.clearPath();
      this.state.pathCoords = [];
    }
    this.ui.showStatus(`${type} point deleted. Syncing...`, "info");
    await this.syncAllPointsToServer();
    this.updateUI();
  }

  async solvePath() {
    if (this.state.isProcessing) return;
    this.setProcessing(true);
    this.ui.showStatus("Computing shortest path...", "info");
    try {
      const data = await this.api.solvePath();
      if (data.success) {
        this.state.pathCoords = data.path.map((p) => [p.lat, p.lng]);
        this.map.drawPath(this.state.pathCoords);
        this.ui.showStatus(data.message, "success");
      } else {
        this.ui.showStatus(data.message, "error");
        this.map.clearPath();
        this.state.pathCoords = [];
      }
    } catch (error) {
      this.ui.showStatus(`Error computing path: ${error.message}`, "error");
    } finally {
      this.setProcessing(false);
      this.updateUI();
    }
  }

  async handleGpxPoints(coords) {
    if (!coords || coords.length === 0) {
      this.ui.showStatus("No points found in the GPX file.", "info");
      return;
    }
    this.ui.showStatus(
      `Found ${coords.length} points in GPX. Adding to planner...`,
      "info",
    );
    await this.clearAll();
    for (let i = 0; i < coords.length; i++) {
      const point = coords[i];
      let type;
      if (i === 0) {
        type = "start";
      } else if (i === coords.length - 1 && coords.length > 1) {
        type = "goal";
      } else {
        type = "intermediate";
      }
      await this.addPoint(point.lat, point.lng, type);
    }
    this.ui.showStatus(
      `Imported ${coords.length} points. Ready to solve.`,
      "success",
    );
    if (this.state.pointCounts.start > 0 && this.state.pointCounts.goal > 0) {
      this.solvePath();
    }
  }

  async clearAll() {
    if (this.state.isProcessing) return;
    try {
      await this.api.clearPoints();
      this.state.markers.forEach((m) => this.map.removeMarker(m.leafletMarker));
      this.map.clearPath();
      this.state.markers = [];
      this.state.pointCounts = { start: 0, goal: 0, intermediate: 0 };
      this.state.pathCoords = [];
      this.ui.showStatus("All points cleared.", "success");
      this.updateUI();
    } catch (error) {
      this.ui.showStatus(`Error clearing points: ${error.message}`, "error");
    }
  }

  async clearPointsByType(type) {
    const markersOfType = this.state.markers.filter((m) => m.type === type);
    markersOfType.forEach((m) => this.map.removeMarker(m.leafletMarker));
    this.state.markers = this.state.markers.filter((m) => m.type !== type);
    this.state.pointCounts[type] = 0;
    await this.syncAllPointsToServer();
    this.updateUI();
  }

  async syncAllPointsToServer() {
    await this.api.clearPoints();
    for (const marker of this.state.markers) {
      const { lat, lng } = marker.leafletMarker.getLatLng();
      const serverResponse = await this.api.addPoint(lat, lng, marker.type);
      marker.id = serverResponse.point_id;
    }
  }

  exportPathToGPX() {
    if (this.state.pathCoords.length === 0) {
      this.ui.showStatus("No path to export.", "error");
      return;
    }
    const trackPoints = this.state.pathCoords
      .map((c) => `<wpt lat="${c[0]}" lon="${c[1]}"></wpt>`)
      .join("\n");
    const gpxData = `<?xml version="1.0" encoding="UTF-8"?><gpx version="1.1" creator="PathSolverApp">${trackPoints}</gpx>`;
    this._downloadFile(gpxData, "path.gpx", "application/gpx+xml");
    this.ui.showStatus("GPX file exported.", "success");
  }

  async sharePathViaWormhole() {
    if (this.state.pathCoords.length === 0) {
      this.ui.showStatus("No path to share.", "error");
      return;
    }

    const gpxData = this._generateGPX();
    const progressDialog = this.ui.showProgressDialog(
      "Creating Wormhole...",
      "Sending path to server...",
    );

    try {
      const data = await this.api.createWormhole(gpxData);
      progressDialog.close();

      if (data.success) {
        this.state.activeTransferId = data.transfer_id;
        this.ui.showWormholeDialog({
          code: data.code,
          command: `wormhole receive ${data.code}`,
          oncancel: () => this.cancelWormholeTransfer(),
        });
      } else {
        this.ui.showErrorDialog({
          title: "Wormhole Failed",
          message: data.message,
          details: data.details,
        });
      }
    } catch (error) {
      progressDialog.close();
      this.ui.showErrorDialog({
        title: "Wormhole Error",
        message: error.message,
        details: "Check backend logs for more info.",
      });
    }
  }

  async cancelWormholeTransfer() {
    if (!this.state.activeTransferId) return;
    try {
      await this.api.cancelWormhole(this.state.activeTransferId);
      this.ui.showStatus("Wormhole transfer cancelled.", "info");
    } catch (error) {
      this.ui.showStatus("Failed to cancel transfer.", "error");
    } finally {
      this.state.activeTransferId = null;
    }
  }

  _generateGPX() {
    const trackPoints = this.state.pathCoords
      .map((c) => `<wpt lat="${c[0]}" lon="${c[1]}"></wpt>`)
      .join("\n");
    return `<?xml version="1.0" encoding="UTF-8"?><gpx version="1.1" creator="PathSolverApp">${trackPoints}</gpx>`;
  }

  _downloadFile(data, filename, type) {
    const blob = new Blob([data], { type });
    const url = URL.createObjectURL(blob);
    const a = Object.assign(document.createElement("a"), {
      href: url,
      download: filename,
    });
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  }
}

document.addEventListener("DOMContentLoaded", () => {
  new PathSolverApp();
});
