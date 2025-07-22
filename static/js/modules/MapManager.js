// static/js/modules/MapManager.js

export class MapManager {
  constructor(mapId, config = {}) {
    this.config = {
      center: [47.5, 13.5],
      initialZoom: 8,
      minZoom: 6,
      maxZoom: 18,
      ...config,
    };
    this.map = L.map(mapId, {
      maxZoom: this.config.maxZoom,
      minZoom: this.config.minZoom,
    }).setView(this.config.center, this.config.initialZoom);

    this.pathLayer = null;
    this.events = new L.Evented();

    this._initLayers();
    this._bindMapEvents();
  }

  on(event, callback) {
    this.events.on(event, callback);
  }

  _parseGpx(gpxString) {
    const parser = new DOMParser();
    const xmlDoc = parser.parseFromString(gpxString, "application/xml");

    // Check if the parser failed due to malformed XML
    const errorNode = xmlDoc.querySelector("parsererror");
    if (errorNode) {
      console.error("GPX Parsing Error:", errorNode.textContent);
      throw new Error("Invalid GPX format. Check the file for syntax errors.");
    }

    let points = [];

    // 1. Prioritize your desired format: a list of Waypoints (<wpt>)
    let nodes = xmlDoc.querySelectorAll("wpt");
    if (nodes.length > 0) {
      nodes.forEach((node) => {
        // Use `getAttribute` which is case-insensitive for HTML but not XML, so this is robust
        const lat = parseFloat(node.getAttribute("lat"));
        const lon = parseFloat(node.getAttribute("lon"));
        if (!isNaN(lat) && !isNaN(lon)) {
          points.push({ lat: lat, lng: lon });
        }
      });
      return points;
    }

    // 2. If no <wpt> tags, fall back to standard trackpoints (<trkpt>)
    nodes = xmlDoc.querySelectorAll("trkpt");
    if (nodes.length > 0) {
      nodes.forEach((node) => {
        const lat = parseFloat(node.getAttribute("lat"));
        const lon = parseFloat(node.getAttribute("lon"));
        if (!isNaN(lat) && !isNaN(lon)) {
          points.push({ lat: lat, lng: lon });
        }
      });
      return points;
    }

    // If we reach here, no usable points were found
    return [];
  }

  /**
   * Parses a GPX file using our custom parser and fires an event.
   * @param {string} gpxData - The text content of the GPX file.
   */
  displayGpxTrack(gpxData) {
    try {
      const coords = this._parseGpx(gpxData);

      if (coords.length === 0) {
        this.events.fire("gpxLoadError", {
          error:
            "No waypoints (<wpt>) or trackpoints (<trkpt>) found in the GPX file.",
        });
        return;
      }

      // Fire a success event with the coordinates for the main app
      this.events.fire("gpxPointsLoaded", { coords });

      // Manually calculate the bounds of the loaded points
      const bounds = L.latLngBounds(coords.map((c) => [c.lat, c.lng]));
      if (bounds.isValid()) {
        this.map.fitBounds(bounds.pad(0.1));
      }
    } catch (error) {
      this.events.fire("gpxLoadError", {
        error:
          error.message ||
          "An unknown error occurred while parsing the GPX file.",
      });
    }
  }

  // --- NO OTHER CHANGES BELOW THIS LINE ---

  createMarker(lat, lng, type) {
    const icon = this._getIcon(type);
    const marker = L.marker([lat, lng], { icon, draggable: true }).addTo(
      this.map,
    );
    this._bindMarkerEvents(marker);
    return marker;
  }

  removeMarker(marker) {
    if (this.map.hasLayer(marker)) {
      this.map.removeLayer(marker);
    }
  }

  drawPath(coords) {
    this.clearPath();
    this.pathLayer = L.polyline(coords, {
      color: "#ff7800",
      weight: 5,
      opacity: 0.8,
    }).addTo(this.map);
    if (this.pathLayer.getBounds().isValid()) {
      this.map.fitBounds(this.pathLayer.getBounds().pad(0.1));
    }
  }

  clearPath() {
    if (this.pathLayer) {
      this.map.removeLayer(this.pathLayer);
      this.pathLayer = null;
    }
  }

  showPopup(latlng, content) {
    L.popup({ closeButton: false, offset: [0, -36] })
      .setLatLng(latlng)
      .setContent(content)
      .openOn(this.map);
  }

  closeAllPopups() {
    this.map.closePopup();
  }

  _initLayers() {
    const osmLayer = L.tileLayer(
      "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png",
      {
        attribution:
          '© <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
      },
    );
    const satelliteLayer = L.tileLayer(
      "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
      {
        attribution: "Tiles © Esri — Source: Esri, i-cubed, a.o.",
      },
    );
    const demLayer = L.tileLayer("/tiles/dem/{z}/{x}/{y}.png", {
      attribution: "Digital Elevation Model © Geoland.at",
      opacity: 0.7,
      maxZoom: 16,
    });
    const baseMaps = { Map: osmLayer, Satellite: satelliteLayer };
    const overlayMaps = { "Elevation Profile": demLayer };
    osmLayer.addTo(this.map);
    demLayer.addTo(this.map);
    L.control.layers(baseMaps, overlayMaps).addTo(this.map);
  }

  _bindMapEvents() {
    this.map.on("click", (e) => this.events.fire("mapClick", e));
  }

  _bindMarkerEvents(marker) {
    marker.on("dragstart", (e) =>
      this.events.fire("markerDragStart", { marker: e.target }),
    );
    marker.on("dragend", (e) =>
      this.events.fire("markerDragEnd", { marker: e.target }),
    );
    marker.on("contextmenu", (e) =>
      this.events.fire("markerRightClick", {
        marker: e.target,
        latlng: e.latlng,
        originalEvent: e.originalEvent,
      }),
    );
  }

  _getIcon(type) {
    const colors = {
      start: "#28a745",
      goal: "#dc3545",
      intermediate: "#007bff",
    };
    const color = colors[type] || "#ffc107";
    const svgIcon = `
        <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" width="36" height="36" style="filter: drop-shadow(0px 3px 3px rgba(0,0,0,0.5));">
            <path d="M12 2C8.13 2 5 5.13 5 9c0 5.25 7 13 7 13s7-7.75 7-13c0-3.87-3.13-7-7-7z" 
                  fill="${color}" stroke="#ffffff" stroke-width="1.5" />
            <circle cx="12" cy="9" r="2.5" fill="#ffffff" />
        </svg>`;
    return L.divIcon({
      html: svgIcon,
      className: "custom-div-icon",
      iconSize: [36, 36],
      iconAnchor: [18, 36],
      popupAnchor: [0, -36],
    });
  }
}
