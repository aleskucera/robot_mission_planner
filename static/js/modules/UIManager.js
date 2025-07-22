// static/js/modules/UIManager.js

export class UIManager {
  constructor() {
    // Cache DOM elements
    this.statusDiv = document.getElementById("status");
    this.pointCountEl = document.getElementById("point-count");
    this.solveBtn = document.getElementById("solve-btn");
    this.exportBtn = document.getElementById("export-btn");
    this.pointBtns = document.querySelectorAll(".point-btn");

    this.activeDialog = null;
    this.activeTransferId = null;
  }

  // --- UI State Management ---

  updatePointCount(pointCounts) {
    const totalPoints = Object.values(pointCounts).reduce((a, b) => a + b, 0);
    if (totalPoints === 0) {
      this.pointCountEl.textContent = "Ready to add points";
    } else {
      const parts = [];
      if (pointCounts.start > 0) parts.push("Start ✓");
      if (pointCounts.goal > 0) parts.push("End ✓");
      if (pointCounts.intermediate > 0)
        parts.push(`${pointCounts.intermediate} stops`);
      this.pointCountEl.textContent = parts.join(" • ");
    }
  }

  setSolveButtonState(enabled) {
    this.solveBtn.disabled = !enabled;
  }

  setExportButtonState(enabled) {
    this.exportBtn.disabled = !enabled;
  }

  setProcessingState(isProcessing, pointCounts) {
    if (isProcessing) {
      this.solveBtn.classList.add("loading");
      this.solveBtn.disabled = true;
    } else {
      this.solveBtn.classList.remove("loading");
      this.setSolveButtonState(pointCounts.start > 0 && pointCounts.goal > 0);
    }
  }

  selectPointMode(mode) {
    this.pointBtns.forEach((btn) => btn.classList.remove("active"));
    if (mode) {
      document.querySelector(`[data-type="${mode}"]`).classList.add("active");
    }
  }

  showStatus(message, type = "info", duration = 3000) {
    this.statusDiv.textContent = message;
    this.statusDiv.className = `status ${type}`;

    if (this.statusTimeout) clearTimeout(this.statusTimeout);
    if (type !== "error") {
      this.statusTimeout = setTimeout(() => {
        this.statusDiv.textContent = "";
        this.statusDiv.className = "status";
      }, duration);
    }
  }

  // --- Dialogs and Menus ---

  showContextMenu({ latlng, ondelete, oncopy }) {
    const container = document.createElement("div");
    container.className = "context-menu";

    const deleteBtn = document.createElement("button");
    deleteBtn.innerHTML = "🗑 Delete";
    deleteBtn.onclick = ondelete;

    const copyBtn = document.createElement("button");
    copyBtn.innerHTML = "📋 Copy Coords";
    copyBtn.onclick = oncopy;

    container.appendChild(deleteBtn);
    container.appendChild(copyBtn);
    return container;
  }

  showProgressDialog(title, message) {
    const { overlay, dialog, close } = this._createBaseDialog(
      title,
      "progress-dialog",
    );

    const messageEl = document.createElement("p");
    messageEl.textContent = message;

    const progressContainer = document.createElement("div");
    progressContainer.className = "progress-container";
    const progressBar = document.createElement("div");
    progressBar.className = "progress-bar";
    progressContainer.appendChild(progressBar);

    const statusEl = document.createElement("p");
    statusEl.className = "status-text";

    dialog.append(messageEl, progressContainer, statusEl);

    let progressInterval = setInterval(() => {
      progressBar.style.width = `${Math.min(100, (parseInt(progressBar.style.width, 10) || 0) + 5)}%`;
    }, 500);

    const updateMessage = (newMessage) => (messageEl.textContent = newMessage);
    const cleanUp = () => {
      clearInterval(progressInterval);
      close();
    };

    return { overlay, updateMessage, close: cleanUp };
  }

  showWormholeDialog({ code, command, oncancel }) {
    const { overlay, dialog, close } = this._createBaseDialog(
      "Share Path via Wormhole",
      "wormhole-dialog",
    );

    const codeEl = Object.assign(document.createElement("p"), {
      textContent: code,
      className: "wormhole-code",
    });
    const instructions = Object.assign(document.createElement("p"), {
      textContent: "Run this command on another computer:",
    });
    const commandBox = Object.assign(document.createElement("div"), {
      textContent: command,
      className: "command-box",
    });
    const copyButton = Object.assign(document.createElement("button"), {
      textContent: "Copy Command",
      className: "copy-command-btn",
    });
    const note = Object.assign(document.createElement("p"), {
      textContent: "This code is valid for a short time.",
      className: "note",
    });
    const cancelButton = Object.assign(document.createElement("button"), {
      textContent: "Cancel Transfer",
      className: "close-dialog-btn cancel-btn",
    });
    const closeButton = Object.assign(document.createElement("button"), {
      textContent: "Close",
      className: "close-dialog-btn",
    });

    copyButton.onclick = async () => {
      await navigator.clipboard.writeText(command);
      copyButton.textContent = "Copied!";
      this.showStatus("Command copied successfully", "success", 2000);
      setTimeout(() => (copyButton.textContent = "Copy Command"), 2000);
    };
    cancelButton.onclick = () => {
      oncancel();
      close();
    };
    closeButton.onclick = close;

    dialog.append(
      codeEl,
      instructions,
      commandBox,
      copyButton,
      note,
      cancelButton,
      closeButton,
    );
  }

  showErrorDialog({ title, message, details }) {
    const { overlay, dialog, close } = this._createBaseDialog(
      title,
      "error-dialog",
    );

    const messageEl = Object.assign(document.createElement("p"), {
      textContent: message,
    });
    const detailsContainer = document.createElement("div");
    detailsContainer.className = "error-details";
    detailsContainer.innerHTML = `<p class="details-title">Technical Details:</p><pre class="details-content">${details || "N/A"}</pre>`;
    const closeButton = Object.assign(document.createElement("button"), {
      textContent: "Close",
      className: "close-dialog-btn",
    });
    closeButton.onclick = close;

    dialog.append(messageEl, detailsContainer, closeButton);
  }

  _createBaseDialog(title, customClass) {
    if (this.activeDialog) {
      document.body.removeChild(this.activeDialog);
    }

    const overlay = document.createElement("div");
    overlay.className = "dialog-overlay";

    const dialog = document.createElement("div");
    dialog.className = `dialog ${customClass}`;

    const titleEl = document.createElement("h3");
    titleEl.textContent = title;

    dialog.appendChild(titleEl);
    overlay.appendChild(dialog);
    document.body.appendChild(overlay);

    this.activeDialog = overlay;
    const close = () => {
      if (document.body.contains(overlay)) {
        document.body.removeChild(overlay);
        this.activeDialog = null;
      }
    };

    return { overlay, dialog, close };
  }
}
