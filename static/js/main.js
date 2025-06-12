class PathSolverApp {
    constructor() {
        this.map = null;
        this.markers = [];
        this.pathPolyline = null;
        this.isProcessing = false;
        this.currentMode = null;
        this.pointCounts = { start: 0, goal: 0, intermediate: 0 };
        this.isDragging = false;
        this.lastDragEndTime = 0;
        this.pathCoords = [];
        this.activeTransferId = null;

        this.init();
    }

    init() {
        this.initializeMap();
        this.bindEventListeners();
        this.updateUI();
        document.getElementById('export-btn').disabled = true;
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

        document.getElementById('export-gpx-btn').addEventListener('click', () => this.exportPathToGPX());
        document.getElementById('export-wormhole-btn').addEventListener('click', () => this.sharePathViaWormhole());

        const exportBtn = document.getElementById('export-btn');
        exportBtn.addEventListener('click', (e) => {
            e.stopPropagation();
            document.querySelector('.export-options').classList.toggle('show');
        });

        document.addEventListener('click', () => {
            document.querySelector('.export-options').classList.remove('show');
        });
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
            `<wpt lat="${coord[0]}" lon="${coord[1]}"></wpt>`
        ).join('\n');

        return `<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="PathSolverApp">
${trackPoints}
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


    async sharePathViaWormhole() {
        if (!this.pathCoords || this.pathCoords.length === 0) {
            this.showStatus('No path to share', 'error');
            return;
        }

        const { overlay, updateMessage, close } = this.showProgressDialog(
            'Creating Wormhole',
            'Generating wormhole code...'
        );

        try {
            const gpxData = this.generateGPX();

            // 15-second timeout for code capture
            const timeoutPromise = new Promise((_, reject) => {
                setTimeout(() => reject(new Error('Failed to capture wormhole code')), 15000);
            });

            updateMessage('Sending path data to server...');
            const fetchPromise = fetch('/create_wormhole', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ gpx: gpxData })
            }).then(response => {
                if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
                return response.json();
            });

            const data = await Promise.race([fetchPromise, timeoutPromise]);

            if (data.success) {
                this.activeTransferId = data.transfer_id;
                const command = `wormhole receive ${data.code}`;
                close();
                this.showWormholeDialog(data.code, command);
                this.showStatus('Wormhole code generated', 'success');
            } else {
                close();
                console.error('Wormhole creation failed:', data);
                this.showErrorDialog(
                    'Wormhole Creation Failed',
                    data.message || 'Failed to create wormhole',
                    data.details ? JSON.stringify(data.details, null, 2) : 'No additional details available'
                );
                this.showStatus('Failed to share path via wormhole', 'error');
            }
        } catch (error) {
            close();
            console.error('Error sharing path via wormhole:', error);
            this.showErrorDialog(
                'Wormhole Creation Error',
                error.message,
                'Check server logs for more details.'
            );
            this.showStatus('Error sharing path via wormhole', 'error');
        }
    }

    showProgressDialog(title, message) {
        const overlay = document.createElement('div');
        overlay.className = 'dialog-overlay';

        const dialog = document.createElement('div');
        dialog.className = 'dialog progress-dialog';

        const titleEl = document.createElement('h3');
        titleEl.textContent = title;

        const messageEl = document.createElement('p');
        messageEl.textContent = message;

        const progressContainer = document.createElement('div');
        progressContainer.className = 'progress-container';

        const progressBar = document.createElement('div');
        progressBar.className = 'progress-bar';
        progressContainer.appendChild(progressBar);

        const statusEl = document.createElement('p');
        statusEl.className = 'status-text';
        statusEl.textContent = 'Starting...';

        const cancelButton = document.createElement('button');
        cancelButton.className = 'close-dialog-btn cancel-btn';
        cancelButton.textContent = 'Cancel';

        dialog.appendChild(titleEl);
        dialog.appendChild(messageEl);
        dialog.appendChild(progressContainer);
        dialog.appendChild(statusEl);
        dialog.appendChild(cancelButton);

        overlay.appendChild(dialog);
        document.body.appendChild(overlay);

        let dots = 0;
        let progress = 0;
        const updateInterval = setInterval(() => {
            dots = (dots + 1) % 4;
            progress = Math.min(100, progress + 5);
            progressBar.style.width = `${progress}%`;
            statusEl.textContent = `Processing${'.'.repeat(dots).padEnd(3)}`;
        }, 500);

        const updateMessage = (newMessage) => {
            messageEl.textContent = newMessage;
        };

        const close = () => {
            clearInterval(updateInterval);
            if (document.body.contains(overlay)) {
                document.body.removeChild(overlay);
            }
        };

        cancelButton.addEventListener('click', () => {
            close();
            if (this.activeTransferId) {
                this.cancelWormholeTransfer(this.activeTransferId);
                this.activeTransferId = null;
            }
            this.showStatus('Wormhole creation cancelled', 'info');
        });

        return { overlay, updateMessage, close };
    }

    async cancelWormholeTransfer(transferId) {
        try {
            const response = await fetch('/cancel_wormhole', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ transfer_id: transferId })
            });
            const data = await response.json();
            if (data.success) {
                this.showStatus('Wormhole transfer cancelled', 'success');
            } else {
                console.error('Failed to cancel wormhole transfer:', data);
                this.showStatus('Failed to cancel wormhole transfer', 'error');
            }
        } catch (error) {
            console.error('Error cancelling wormhole transfer:', error);
            this.showStatus('Error cancelling wormhole transfer', 'error');
        }
    }

    showWormholeDialog(code, command) {
        const overlay = document.createElement('div');
        overlay.className = 'dialog-overlay';

        const dialog = document.createElement('div');
        dialog.className = 'dialog wormhole-dialog';

        const title = document.createElement('h3');
        title.textContent = 'Share Path';

        const codeEl = document.createElement('p');
        codeEl.textContent = code;
        codeEl.className = 'wormhole-code';

        const instructions = document.createElement('p');
        instructions.textContent = 'Run this command on another computer to download the path:';

        const commandBox = document.createElement('div');
        commandBox.className = 'command-box';
        commandBox.textContent = command;

        const copyButton = document.createElement('button');
        copyButton.className = 'copy-command-btn';
        copyButton.textContent = 'Copy Command';
        copyButton.addEventListener('click', () => {
            navigator.clipboard.writeText(command)
                .then(() => {
                    copyButton.textContent = 'Copied!';
                    setTimeout(() => copyButton.textContent = 'Copy Command', 2000);
                    this.showStatus('Command copied to clipboard', 'success');
                })
                .catch(err => {
                    console.error('Failed to copy command:', err);
                    this.showStatus('Failed to copy command', 'error');
                });
        });

        const note = document.createElement('p');
        note.className = 'note';
        note.textContent = 'The transfer is active for up to 60 seconds or until completed.';

        const cancelButton = document.createElement('button');
        cancelButton.className = 'close-dialog-btn cancel-btn';
        cancelButton.textContent = 'Cancel Transfer';
        cancelButton.addEventListener('click', () => {
            if (this.activeTransferId) {
                this.cancelWormholeTransfer(this.activeTransferId);
                this.activeTransferId = null;
            }
            document.body.removeChild(overlay);
        });

        const closeButton = document.createElement('button');
        closeButton.className = 'close-dialog-btn';
        closeButton.textContent = 'Close';
        closeButton.addEventListener('click', () => {
            document.body.removeChild(overlay);
        });

        dialog.appendChild(title);
        dialog.appendChild(codeEl);
        dialog.appendChild(instructions);
        dialog.appendChild(commandBox);
        dialog.appendChild(copyButton);
        dialog.appendChild(note);
        dialog.appendChild(cancelButton);
        dialog.appendChild(closeButton);

        overlay.appendChild(dialog);
        document.body.appendChild(overlay);
    }

    showErrorDialog(title, message, details) {
        const overlay = document.createElement('div');
        overlay.className = 'dialog-overlay';

        const dialog = document.createElement('div');
        dialog.className = 'dialog error-dialog';

        const titleEl = document.createElement('h3');
        titleEl.textContent = title;

        const messageEl = document.createElement('p');
        messageEl.textContent = message;

        const detailsContainer = document.createElement('div');
        detailsContainer.className = 'error-details';

        const detailsTitle = document.createElement('p');
        detailsTitle.className = 'details-title';
        detailsTitle.textContent = 'Technical Details:';

        const detailsContent = document.createElement('pre');
        detailsContent.className = 'details-content';
        detailsContent.textContent = details;

        detailsContainer.appendChild(detailsTitle);
        detailsContainer.appendChild(detailsContent);

        const closeButton = document.createElement('button');
        closeButton.className = 'close-dialog-btn';
        closeButton.textContent = 'Close';
        closeButton.addEventListener('click', () => {
            document.body.removeChild(overlay);
        });

        dialog.appendChild(titleEl);
        dialog.appendChild(messageEl);
        dialog.appendChild(detailsContainer);
        dialog.appendChild(closeButton);

        overlay.appendChild(dialog);
        document.body.appendChild(overlay);
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

