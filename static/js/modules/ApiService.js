// static/js/modules/ApiService.js

const API_BASE_URL = "/api"; // Corresponds to the Blueprint url_prefix in Flask

export const ApiService = {
  async addPoint(lat, lng, type) {
    return this._post(`${API_BASE_URL}/add_point`, { lat, lng, type });
  },

  async solvePath() {
    return this._post(`${API_BASE_URL}/solve_path`);
  },

  async clearPoints() {
    return this._post(`${API_BASE_URL}/clear_points`);
  },

  async createWormhole(gpx) {
    return this._post(`${API_BASE_URL}/create_wormhole`, { gpx });
  },

  async cancelWormhole(transfer_id) {
    return this._post(`${API_BASE_URL}/cancel_wormhole`, { transfer_id });
  },

  async _post(url, body = {}) {
    try {
      const response = await fetch(url, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      if (!response.ok) {
        // Try to get a more specific error message from the server's response
        const errorData = await response
          .json()
          .catch(() => ({ message: "No error details from server." }));
        throw new Error(
          `HTTP error! status: ${response.status}. Message: ${errorData.message || "Unknown error"}`,
        );
      }
      return await response.json();
    } catch (error) {
      console.error(`API call to ${url} failed:`, error);
      // Re-throw the error so the calling component in the main app can handle it
      throw error;
    }
  },
};
