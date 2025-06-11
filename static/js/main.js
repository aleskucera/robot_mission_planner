class PathSolverApp {
    constructor() {
        this.map = null;
        this.markers = [];
        this.pathPolyline = null;
        this.isProcessing = false;
        this.currentMode = null;
        this.pointCounts = { start: 0, goal: 0, intermediate: 0 };
        this.isDragging = false;
        this.lastDragEndTime = 0; // Track time of last drag end
        this.pathCoords = []; // Initialize path coordinates for export

        this.init();
    }

    init() {
        this.initializeMap();
        this.bindEventListeners();
        this.updateUI();
        document.getElementById('export-btn').disabled = true; // Initially disable export button
    }

    initializeMap() {
        this.map = L.map('map', {
            center: [50.07644719992767, 14.418223288038638],
            zoom: 13,
            maxZoom: 18,
            minZoom: 2,
            zoomControl: true,
            scrollWheelZoom: true,
            doubleClickZoom: true,
            touchZoom: true,
            zoomSnap: 0.5,
            zoomDelta: 0.5
        });

        const osmLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors',
            maxZoom: 18
        });

        const satelliteLayer = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
            attribution: '© Esri, DigitalGlobe, GeoEye, Earthstar Geographics, CNES/Airbus DS, USDA, USGS, AeroGRID, IGN, and the GIS User Community',
            maxZoom: 18
        });

        const cartoDBLayer = L.tileLayer('https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png', {
            attribution: '© OpenStreetMap contributors, © CartoDB',
            maxZoom: 18,
            subdomains: 'abcd'
        });

        const osmDeLayer = L.tileLayer('https://{s}.tile.openstreetmap.de/tiles/osmde/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors',
            maxZoom: 18
        });

        osmLayer.addTo(this.map);

        const baseMaps = {
            "Street Map": osmLayer,
            "Clean Style": cartoDBLayer,
            "Detailed Street": osmDeLayer,
            "Satellite": satelliteLayer
        };

        L.control.layers(baseMaps).addTo(this.map);
        this.addZoomIndicator();
        this.map.on('click', (e) => this.handleMapClick(e));
        this.addCustomZoomControls();
    }

    bindEventListeners() {
        const pointBtns = document.querySelectorAll('.point-btn');
        pointBtns.forEach(btn => {
            btn.addEventListener('click', () => this.selectPointMode(btn.dataset.type));
        });

        document.getElementById('solve-btn').addEventListener('click', () => this.solvePath());
        document.getElementById('clear-btn').addEventListener('click', () => this.clearPoints());
        document.getElementById('export-btn').addEventListener('click', () => this.exportPathToGPX());
    }

    selectPointMode(mode) {
        document.querySelectorAll('.point-btn').forEach(btn => btn.classList.remove('active'));
        document.querySelector(`[data-type="${mode}"]`).classList.add('active');

        this.currentMode = mode;
        this.showStatus(`Click on map to place ${mode} point`, 'info');
    }

    async handleMapClick(e) {
        if (!this.currentMode || this.isProcessing || this.isDragging || (Date.now() - this.lastDragEndTime < 200)) return;

        const lat = e.latlng.lat;
        const lng = e.latlng.lng;

        if (this.currentMode !== 'intermediate') {
            await this.removePointsByType(this.currentMode);
        }

        this.addPoint(lat, lng, this.currentMode);
    }

    async removePointsByType(type) {
        this.markers = this.markers.filter(marker => {
            if (marker.pointType === type) {
                this.map.removeLayer(marker);
                return false;
            }
            return true;
        });

        this.pointCounts[type] = 0;

        if (this.pathPolyline) {
            this.map.removeLayer(this.pathPolyline);
            this.pathPolyline = null;
            document.getElementById('export-btn').disabled = true; // Disable export button when path is removed
        }

        try {
            await fetch('/clear_points', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' }
            });

            for (const marker of this.markers) {
                await this.sendPointToServer(marker.getLatLng().lat, marker.getLatLng().lng, marker.pointType);
            }
        } catch (error) {
            console.error('Error updating server:', error);
        }
    }

    async addPoint(lat, lng, pointType) {
        try {
            const marker = this.createDraggableMarker(lat, lng, pointType);
            marker.pointType = pointType;
            this.markers.push(marker);

            const response = await this.sendPointToServer(lat, lng, pointType);

            if (response.success) {
                this.pointCounts[pointType]++;
                this.updateUI();

                this.setupMarkerInteractions(marker, pointType);

                this.showStatus(`${this.capitalizeFirst(pointType)} point added`, 'success');
            } else {
                this.map.removeLayer(marker);
                this.markers.splice(this.markers.indexOf(marker), 1);
                this.showStatus('Failed to add point', 'error');
            }
        } catch (error) {
            console.error('Error adding point:', error);
            this.showStatus('Error adding point', 'error');
        }
    }

    createDraggableMarker(lat, lng, type) {
        const markerConfigs = {
            start: { className: 'custom-start-marker', size: [16, 16], color: '#27ae60' },
            goal: { className: 'custom-goal-marker', size: [16, 16], color: '#e74c3c' },
            intermediate: { className: 'custom-intermediate-marker', size: [12, 12], color: '#3498db' }
        };

        const config = markerConfigs[type];
        const icon = L.divIcon({
            className: 'custom-marker draggable-marker',
            html: `
                <div class="${config.className}" style="position: relative;">
                    <div class="drag-handle" title="Drag to move">⋮⋮</div>
                </div>
            `,
            iconSize: config.size,
            iconAnchor: [config.size[0]/2, config.size[1]/2]
        });

        const marker = L.marker([lat, lng], {
            icon: icon,
            draggable: true,
            autoPan: true
        }).addTo(this.map);

        this.setupDragEvents(marker, type);

        return marker;
    }

    setupDragEvents(marker, type) {
        marker.on('dragstart', (e) => {
            this.isDragging = true;
            this.showStatus(`Dragging ${type} point...`, 'info');

            if (this.pathPolyline) {
                this.pathPolyline.setStyle({ opacity: 0.3 });
            }
        });

        marker.on('dragend', async (e) => {
            this.isDragging = false;
            this.lastDragEndTime = Date.now();
            const newLatLng = e.target.getLatLng();

            try {
                await this.updatePointPosition(marker, newLatLng.lat, newLatLng.lng);
                this.showStatus(`${this.capitalizeFirst(type)} point moved`, 'success');

                if (this.pathPolyline) {
                    this.pathPolyline.setStyle({ opacity: 0.8 });
                }
            } catch (error) {
                console.error('Error updating point position:', error);
                this.showStatus('Error updating point position', 'error');
            }
        });

        marker.on('drag', (e) => {
            // Optional: Update tooltip position during drag
        });
    }

    setupMarkerInteractions(marker, pointType) {
        const tooltipText = pointType === 'intermediate' ? 'Stop' :
                          pointType === 'start' ? 'Start' : 'End';
        marker.bindTooltip(tooltipText, {
            permanent: false,
            direction: 'top',
            className: 'marker-tooltip'
        });

        marker.on('contextmenu', (e) => {
            L.DomEvent.preventDefault(e);
            this.showContextMenu(e.latlng, marker);
        });
    }

    async deleteMarker(marker) {
        try {
            this.map.removeLayer(marker);
            this.markers = this.markers.filter(m => m !== marker);
            this.pointCounts[marker.pointType]--;

            if ((marker.pointType === 'start' || marker.pointType === 'goal') && this.pathPolyline) {
                this.map.removeLayer(this.pathPolyline);
                this.pathPolyline = null;
                document.getElementById('export-btn').disabled = true; // Disable export button when path is removed
            }

            await this.syncPointsWithServer();

            this.updateUI();
            this.showStatus(`${this.capitalizeFirst(marker.pointType)} point deleted`, 'success');
        } catch (error) {
            console.error('Error deleting point:', error);
            this.showStatus('Error deleting point', 'error');
        }
    }

    showContextMenu(latlng, marker) {
        const container = document.createElement('div');
        container.className = 'context-menu';

        const deleteBtn = document.createElement('button');
        deleteBtn.className = 'context-btn delete-btn';
        deleteBtn.innerHTML = '🗑️ Delete Point';
        deleteBtn.addEventListener('click', () => {
            this.deleteMarker(marker);
            this.map.closePopup();
        });

        const copyBtn = document.createElement('button');
        copyBtn.className = 'context-btn copy-btn';
        copyBtn.innerHTML = '📋 Copy Coordinates';
        copyBtn.addEventListener('click', () => {
            this.copyCoordinates(marker);
            this.map.closePopup();
        });

        container.appendChild(deleteBtn);
        container.appendChild(copyBtn);

        L.popup({
            closeButton: false,
            autoClose: true,
            className: 'context-menu-popup'
        })
        .setLatLng(latlng)
        .setContent(container)
        .openOn(this.map);
    }

    async copyCoordinates(marker) {
        const latlng = marker.getLatLng();
        const coords = `${latlng.lat.toFixed(6)}, ${latlng.lng.toFixed(6)}`;

        try {
            await navigator.clipboard.writeText(coords);
            this.showStatus('Coordinates copied to clipboard', 'success');
        } catch (err) {
            console.error('Failed to copy coordinates:', err);
            this.showStatus('Failed to copy coordinates', 'error');
        }
        this.map.closePopup();
    }

    async updatePointPosition(marker, lat, lng) {
        await this.syncPointsWithServer();
    }

    async syncPointsWithServer() {
        try {
            await fetch('/clear_points', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' }
            });

            for (const marker of this.markers) {
                const latlng = marker.getLatLng();
                await this.sendPointToServer(latlng.lat, latlng.lng, marker.pointType);
            }
        } catch (error) {
            console.error('Error syncing with server:', error);
            throw error;
        }
    }

    async sendPointToServer(lat, lng, pointType) {
        const response = await fetch('/add_point', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ lat, lng, type: pointType })
        });

        if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
        return await response.json();
    }

    addZoomIndicator() {
        const zoomIndicator = L.control({position: 'bottomleft'});

        zoomIndicator.onAdd = (map) => {
            const div = L.DomUtil.create('div', 'zoom-indicator');
            div.innerHTML = `<span id="zoom-level">Zoom: ${map.getZoom().toFixed(1)}/18</span>`;
            return div;
        };

        zoomIndicator.addTo(this.map);

        this.map.on('zoomend', () => {
            const zoomLevel = document.getElementById('zoom-level');
            if (zoomLevel) {
                zoomLevel.textContent = `Zoom: ${this.map.getZoom().toFixed(1)}/18`;
            }
        });
    }

    addCustomZoomControls() {
        const zoomToMax = L.control({position: 'topright'});
        zoomToMax.onAdd = () => {
            const div = L.DomUtil.create('div', 'custom-zoom-control');
            div.innerHTML = '<button class="zoom-btn zoom-max" title="Zoom to maximum">🔍MAX</button>';

            L.DomEvent.on(div.querySelector('.zoom-max'), 'click', (e) => {
                L.DomEvent.stopPropagation(e);
                this.map.setZoom(18);
            });

            return div;
        };

        const zoomToOverview = L.control({position: 'topright'});
        zoomToOverview.onAdd = () => {
            const div = L.DomUtil.create('div', 'custom-zoom-control');
            div.innerHTML = '<button class="zoom-btn zoom-overview" title="Zoom to overview">🌍</button>';

            L.DomEvent.on(div.querySelector('.zoom-overview'), 'click', (e) => {
                L.DomEvent.stopPropagation(e);
                this.map.setZoom(10);
            });

            return div;
        };

        zoomToMax.addTo(this.map);
        zoomToOverview.addTo(this.map);
    }

    async solvePath() {
        if (this.pointCounts.start === 0 || this.pointCounts.goal === 0) {
            this.showStatus('Need both start and end points', 'error');
            document.getElementById('export-btn').disabled = true; // Ensure export button is disabled
            return;
        }

        if (this.isProcessing) return;

        try {
            this.setProcessingState(true);
            this.showStatus('Computing shortest path...', 'info');

            const response = await fetch('/solve_path', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' }
            });

            if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
            const data = await response.json();

            if (data.success) {
                this.drawPath(data.path);
                this.showStatus(data.message, 'success');
                document.getElementById('export-btn').disabled = false; // Enable export button on success
            } else {
                this.showStatus(data.message, 'error');
                document.getElementById('export-btn').disabled = true; // Disable export button on failure
            }
        } catch (error) {
            console.error('Error solving path:', error);
            this.showStatus('Error computing path', 'error');
            document.getElementById('export-btn').disabled = true; // Disable export button on error
        } finally {
            this.setProcessingState(false);
        }
    }

    drawPath(path) {
        if (this.pathPolyline) {
            this.map.removeLayer(this.pathPolyline);
        }

        this.pathCoords = path.map(point => [point.lat, point.lng]);
        this.pathPolyline = L.polyline(this.pathCoords, {
            color: '#e74c3c',
            weight: 3,
            opacity: 0.8,
            smoothFactor: 1
        }).addTo(this.map);

        this.map.fitBounds(this.pathPolyline.getBounds(), { padding: [20, 20] });
    }

    exportPathToGPX() {
        if (!this.pathCoords || this.pathCoords.length === 0) {
            this.showStatus('No path to export', 'error');
            return;
        }

        const gpxData = this.generateGPX();
        this.downloadFile(gpxData, 'path.gpx', 'application/gpx+xml');
        this.showStatus('GPX file exported successfully', 'success');
    }

    generateGPX() {
        const trackPoints = this.pathCoords.map(coord =>
            `<trkpt lat="${coord[0]}" lon="${coord[1]}"></trkpt>`
        ).join('\n');

        return `<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="PathSolverApp">
    <trk>
        <name>Computed Path</name>
        <trkseg>
            ${trackPoints}
        </trkseg>
    </trk>
</gpx>`;
    }

    downloadFile(data, filename, type) {
        const blob = new Blob([data], { type });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = filename;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);
    }

    async clearPoints() {
        if (this.isProcessing) return;

        try {
            this.markers.forEach(marker => this.map.removeLayer(marker));
            this.markers = [];

            if (this.pathPolyline) {
                this.map.removeLayer(this.pathPolyline);
                this.pathPolyline = null;
                document.getElementById('export-btn').disabled = true; // Disable export button when path is cleared
            }

            this.pointCounts = { start: 0, goal: 0, intermediate: 0 };
            this.currentMode = null;

            document.querySelectorAll('.point-btn').forEach(btn => btn.classList.remove('active'));

            const response = await fetch('/clear_points', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' }
            });

            if (response.ok) {
                this.updateUI();
                this.showStatus('All points cleared', 'success');
            }
        } catch (error) {
            console.error('Error clearing points:', error);
            this.showStatus('Error clearing points', 'error');
        }
    }

    updateUI() {
        const totalPoints = Object.values(this.pointCounts).reduce((a, b) => a + b, 0);
        const pointCountEl = document.getElementById('point-count');

        if (totalPoints === 0) {
            pointCountEl.textContent = 'Ready to add points';
        } else {
            const parts = [];
            if (this.pointCounts.start > 0) parts.push('Start ✓');
            if (this.pointCounts.goal > 0) parts.push('End ✓');
            if (this.pointCounts.intermediate > 0) parts.push(`${this.pointCounts.intermediate} stops`);
            pointCountEl.textContent = parts.join(' • ');
        }

        const solveBtn = document.getElementById('solve-btn');
        solveBtn.disabled = this.pointCounts.start === 0 || this.pointCounts.goal === 0;
    }

    showStatus(message, type) {
        const statusDiv = document.getElementById('status');
        statusDiv.textContent = message;
        statusDiv.className = `status ${type}`;

        if (type === 'success' || type === 'info') {
            setTimeout(() => {
                statusDiv.textContent = '';
                statusDiv.className = 'status';
            }, 3000);
        }
    }

    setProcessingState(isProcessing) {
        this.isProcessing = isProcessing;
        const solveBtn = document.getElementById('solve-btn');

        if (isProcessing) {
            solveBtn.classList.add('loading');
            solveBtn.disabled = true;
        } else {
            solveBtn.classList.remove('loading');
            solveBtn.disabled = this.pointCounts.start === 0 || this.pointCounts.goal === 0;
        }
    }

    capitalizeFirst(str) {
        return str.charAt(0).toUpperCase() + str.slice(1);
    }
}

let app;
document.addEventListener('DOMContentLoaded', () => {
    app = new PathSolverApp();
});