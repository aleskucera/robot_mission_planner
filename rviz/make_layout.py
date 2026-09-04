#!/usr/bin/env python3
"""Regenerate the "QMainWindow State" line of robotour.rviz.

rviz stores its dock arrangement as a hex dump of QMainWindow::saveState(), which
cannot sensibly be written by hand. This rebuilds the same window out of empty Qt
docks -- their objectName is what rviz matches, and for an Image display that is
the display's Name -- arranges them, and prints the hex.

    ~/Work/helhest-singularity/exec.sh python3 rviz/make_layout.py [--write robotour.rviz]

Layout: Displays (+ Views tabbed behind it) down the left, the two camera panels
side by side across the top, the 3D view filling the rest. Qt gives the top-left
corner to the left dock area, so the Displays panel spans the full height and the
images sit above the 3D view only.
"""

import argparse
import os
import re
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PyQt5.QtCore import Qt  # noqa: E402
from PyQt5.QtWidgets import QApplication, QDockWidget, QMainWindow, QWidget  # noqa: E402

WIDTH, HEIGHT = 1920, 1080
SIDE_WIDTH = 360      # Displays / Views column
IMAGE_HEIGHT = 400    # the two camera panels
# Panel names, and the Name: of the two Image displays, exactly as robotour.rviz spells them.
SIDE = ["Displays", "Views"]
IMAGES = ["Odin camera", "Segmented path"]


def build() -> str:
    app = QApplication(sys.argv[:1])
    win = QMainWindow()
    win.resize(WIDTH, HEIGHT)
    win.setCentralWidget(QWidget())

    docks = {}
    for name in SIDE + IMAGES:
        dock = QDockWidget(name, win)
        dock.setObjectName(name)
        dock.setWidget(QWidget())
        docks[name] = dock

    win.addDockWidget(Qt.LeftDockWidgetArea, docks["Displays"])
    win.tabifyDockWidget(docks["Displays"], docks["Views"])
    win.addDockWidget(Qt.TopDockWidgetArea, docks[IMAGES[0]])
    win.splitDockWidget(docks[IMAGES[0]], docks[IMAGES[1]], Qt.Horizontal)

    win.show()
    app.processEvents()
    win.resizeDocks([docks["Displays"]], [SIDE_WIDTH], Qt.Horizontal)
    win.resizeDocks([docks[n] for n in IMAGES], [2, 2], Qt.Horizontal)  # equal halves
    win.resizeDocks([docks[n] for n in IMAGES], [IMAGE_HEIGHT] * 2, Qt.Vertical)
    docks["Displays"].raise_()
    app.processEvents()

    state = bytes(win.saveState().toHex()).decode()
    app.quit()
    return state


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", help="rviz config to patch in place")
    args = ap.parse_args()
    state = build()
    if not args.write:
        print(state)
        return
    path = args.write if os.path.isabs(args.write) else os.path.join(os.path.dirname(__file__), args.write)
    with open(path) as f:
        config = f.read()
    patched, n = re.subn(r"(?m)^(  QMainWindow State: ).*$", r"\g<1>" + state, config)
    if n != 1:
        sys.exit(f"{path}: expected one 'QMainWindow State:' line, found {n}")
    with open(path, "w") as f:
        f.write(patched)
    print(f"{path}: window state updated ({len(state)} hex chars)")


if __name__ == "__main__":
    main()
