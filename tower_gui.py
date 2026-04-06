#!/usr/bin/env python3
"""
Tower Placement GUI — Qt Edition
─────────────────────────────────
• QProcess async solver (non-blocking)
• QPixmap-cached grid rendering
• Live SINR heatmap overlay
• Dark-mode industrial QSS
"""

import sys
import os
import csv
import glob
import math
import random
import numpy as np
from pathlib import Path

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QSpinBox, QDoubleSpinBox,
    QCheckBox, QRadioButton, QButtonGroup, QTextEdit, QFileDialog,
    QMessageBox, QScrollArea, QFrame, QSplitter, QGroupBox,
    QProgressBar, QSlider, QToolTip, QSizePolicy, QStackedWidget,
)
from PyQt6.QtCore import (
    Qt, QProcess, QTimer, QRectF, QPointF, QSize, pyqtSignal, QObject,
)
from PyQt6.QtGui import (
    QPixmap, QPainter, QColor, QPen, QBrush, QFont, QFontDatabase,
    QLinearGradient, QRadialGradient, QPainterPath, QImage, QWheelEvent,
    QMouseEvent, QCursor, QIcon, QPalette,
)

# ── Constants ──────────────────────────────────────────────────────────
TOWER_BINARY = "./tower"
NOISE_FLOOR = 1e-4
PATH_LOSS_EXP = 3.5
MAX_CANVAS = 700

POWERS_SMALL = {"Macro": 0.15, "Micro": 0.02, "Pico": 0.004}
POWERS_BIG = {"Macro": 600, "Micro": 150, "Pico": 25}
GIVEN_RADII_BIG = {"Macro": 81, "Micro": 54, "Pico": 37}

DEFAULT_TOWER_TYPES = [
    {"name": "Macro", "power": 0.15, "cost": 5000, "capacity": 30},
    {"name": "Micro", "power": 0.02, "cost": 2500, "capacity": 15},
    {"name": "Pico", "power": 0.004, "cost": 1000, "capacity": 8},
]

TOWER_COLORS = {
    "Macro": "#FF4757",
    "Micro": "#1E90FF",
    "Pico": "#2ED573",
}

PALETTE_FALLBACK = [
    "#FF6348", "#3742FA", "#2ED573", "#FFA502",
    "#A55EEA", "#1ABC9C", "#E67E22", "#636E72",
]

# ── Dark-mode QSS ─────────────────────────────────────────────────────
DARK_QSS = """
QMainWindow, QWidget {
    background-color: #1a1a2e;
    color: #e0e0e0;
    font-family: 'Segoe UI', 'SF Pro Display', 'Helvetica Neue', sans-serif;
    font-size: 13px;
}

QGroupBox {
    border: 1px solid #2a2a4a;
    border-radius: 6px;
    margin-top: 14px;
    padding: 14px 10px 10px 10px;
    font-weight: bold;
    font-size: 12px;
    color: #7f8fa6;
    letter-spacing: 1.5px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 12px;
    padding: 0 6px;
}

QPushButton {
    background-color: #2a2a4a;
    border: 1px solid #3a3a5a;
    border-radius: 5px;
    padding: 7px 14px;
    color: #e0e0e0;
    font-weight: 500;
}
QPushButton:hover {
    background-color: #3a3a5a;
    border-color: #5a5a8a;
}
QPushButton:pressed {
    background-color: #4a4a6a;
}
QPushButton#runSolver {
    background-color: #e74c3c;
    border: none;
    color: white;
    font-weight: bold;
    font-size: 13px;
    padding: 10px;
}
QPushButton#runSolver:hover {
    background-color: #ff6b6b;
}
QPushButton#runSolver:disabled {
    background-color: #555;
    color: #888;
}
QPushButton#viewBig {
    background-color: #1e90ff;
    border: none;
    color: white;
    font-weight: bold;
}
QPushButton#viewBig:hover {
    background-color: #4dabff;
}

QComboBox {
    background-color: #2a2a4a;
    border: 1px solid #3a3a5a;
    border-radius: 4px;
    padding: 5px 10px;
    color: #e0e0e0;
    min-height: 22px;
}
QComboBox::drop-down {
    border: none;
    width: 20px;
}
QComboBox QAbstractItemView {
    background-color: #2a2a4a;
    border: 1px solid #3a3a5a;
    color: #e0e0e0;
    selection-background-color: #4a4a6a;
}

QSpinBox, QDoubleSpinBox, QLineEdit {
    background-color: #16213e;
    border: 1px solid #2a2a4a;
    border-radius: 4px;
    padding: 4px 8px;
    color: #e0e0e0;
}
QSpinBox:focus, QDoubleSpinBox:focus, QLineEdit:focus {
    border-color: #4a90d9;
}

QTextEdit {
    background-color: #16213e;
    border: 1px solid #2a2a4a;
    border-radius: 6px;
    padding: 8px;
    color: #c8d6e5;
    font-family: 'JetBrains Mono', 'Fira Code', 'Consolas', monospace;
    font-size: 12px;
}

QCheckBox, QRadioButton {
    color: #c8d6e5;
    spacing: 6px;
}
QCheckBox::indicator, QRadioButton::indicator {
    width: 16px;
    height: 16px;
}

QProgressBar {
    border: 1px solid #2a2a4a;
    border-radius: 4px;
    background-color: #16213e;
    text-align: center;
    color: #e0e0e0;
    height: 18px;
}
QProgressBar::chunk {
    background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,
        stop:0 #e74c3c, stop:1 #ff6b6b);
    border-radius: 3px;
}

QScrollBar:vertical {
    background: #1a1a2e;
    width: 10px;
    border: none;
}
QScrollBar::handle:vertical {
    background: #3a3a5a;
    border-radius: 5px;
    min-height: 30px;
}
QScrollBar::handle:vertical:hover {
    background: #5a5a8a;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0;
}

QScrollBar:horizontal {
    background: #1a1a2e;
    height: 10px;
    border: none;
}
QScrollBar::handle:horizontal {
    background: #3a3a5a;
    border-radius: 5px;
    min-width: 30px;
}

QSlider::groove:horizontal {
    height: 4px;
    background: #2a2a4a;
    border-radius: 2px;
}
QSlider::handle:horizontal {
    width: 16px;
    height: 16px;
    margin: -6px 0;
    background: #4a90d9;
    border-radius: 8px;
}
QSlider::handle:horizontal:hover {
    background: #6ab0ff;
}

QSplitter::handle {
    background-color: #2a2a4a;
    width: 2px;
}

QLabel#sectionHeader {
    color: #7f8fa6;
    font-weight: bold;
    font-size: 11px;
    letter-spacing: 1.5px;
    padding: 4px 0;
}

QLabel#statusLabel {
    color: #7f8fa6;
    font-size: 11px;
    padding: 4px;
}

QFrame#separator {
    background-color: #2a2a4a;
    max-height: 1px;
    margin: 6px 0;
}
"""


def effective_radius(power):
    thr = NOISE_FLOOR * 1.01
    return (power / thr) ** (1.0 / PATH_LOSS_EXP)


def read_csv_file(p):
    if not os.path.exists(p):
        return []
    with open(p) as f:
        return list(csv.DictReader(f))


def make_separator():
    sep = QFrame()
    sep.setObjectName("separator")
    sep.setFrameShape(QFrame.Shape.HLine)
    sep.setFixedHeight(1)
    return sep


def section_label(text):
    lbl = QLabel(text)
    lbl.setObjectName("sectionHeader")
    return lbl


# ══════════════════════════════════════════════════════════════════════
#  GRID CANVAS — QWidget with QPixmap caching & SINR overlay
# ══════════════════════════════════════════════════════════════════════
class GridCanvas(QWidget):
    """High-performance grid renderer with cached pixmap and SINR heatmap."""

    cell_hovered = pyqtSignal(int, int)  # (col, row)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self.setMouseTracking(True)
        self.setCursor(QCursor(Qt.CursorShape.CrossCursor))

        # Grid data
        self.rows = 10
        self.cols = 10
        self.grid_buildings = []
        self.grid_demand = []
        self.cell_size = 40

        # Rendering caches
        self._base_pixmap = None  # QPixmap of grid (buildings + demand)
        self._sinr_pixmap = None  # QPixmap of SINR heatmap overlay

        # Tower overlay data
        self.towers = []  # list of dicts {x, y, name}
        self.tower_types = list(DEFAULT_TOWER_TYPES)
        self.show_eff_radii = True
        self.show_given_radii = True
        self.show_sinr = False

        # Zoom / pan
        self._zoom = 1.0
        self._pan_offset = QPointF(20, 20)
        self._panning = False
        self._last_pan_pos = QPointF()

        # Editing
        self.tool = "building"
        self.demand_value = 5
        self.editable = True

    def set_grid(self, rows, cols, buildings=None, demand=None):
        self.rows = rows
        self.cols = cols
        if buildings is not None:
            self.grid_buildings = buildings
        else:
            self.grid_buildings = [[False] * cols for _ in range(rows)]
        if demand is not None:
            self.grid_demand = demand
        else:
            self.grid_demand = [[0.0] * cols for _ in range(rows)]

        self.cell_size = min(40, MAX_CANVAS // max(rows, cols))
        self.cell_size = max(self.cell_size, 4)
        self.towers = []
        self.invalidate_base()
        self.update()

    def invalidate_base(self):
        self._base_pixmap = None
        self._sinr_pixmap = None

    def invalidate_sinr(self):
        self._sinr_pixmap = None

    # ── Base pixmap build ──────────────────────────────────────────────
    def _build_base_pixmap(self):
        cs = self.cell_size
        w, h = self.cols * cs, self.rows * cs
        img = QImage(w, h, QImage.Format.Format_RGB32)

        max_dem = 1.0
        for j in range(self.rows):
            for i in range(self.cols):
                if self.grid_demand[j][i] > max_dem:
                    max_dem = self.grid_demand[j][i]

        for j in range(self.rows):
            for i in range(self.cols):
                if self.grid_buildings[j][i]:
                    color = QColor(50, 50, 65)
                elif self.grid_demand[j][i] > 0:
                    intensity = self.grid_demand[j][i] / max_dem
                    r = int(30 + intensity * 40)
                    g = int(30 + intensity * 20)
                    b = int(60 + intensity * 80)
                    color = QColor(r, g, b)
                else:
                    color = QColor(26, 26, 46)

                # Fill cell block
                for dy in range(cs):
                    for dx in range(cs):
                        img.setPixelColor(i * cs + dx, j * cs + dy, color)

        self._base_pixmap = QPixmap.fromImage(img)

    # ── SINR heatmap ──────────────────────────────────────────────────
    def _build_sinr_pixmap(self):
        """Compute SINR at each cell center from current tower placements."""
        cs = self.cell_size
        w, h = self.cols * cs, self.rows * cs
        img = QImage(w, h, QImage.Format.Format_ARGB32)
        img.fill(QColor(0, 0, 0, 0))

        if not self.towers:
            self._sinr_pixmap = QPixmap.fromImage(img)
            return

        # Build power map
        power_map = {tt["name"]: tt["power"] for tt in self.tower_types}

        # Pre-compute tower positions and powers
        tower_positions = []
        for t in self.towers:
            tx, ty = float(t["x"]) + 0.5, float(t["y"]) + 0.5
            pw = power_map.get(t["name"], 0.01)
            tower_positions.append((tx, ty, pw))

        if not tower_positions:
            self._sinr_pixmap = QPixmap.fromImage(img)
            return

        # Use numpy for vectorized SINR computation
        # Sample at cell centers
        cx = np.arange(self.cols) + 0.5
        cy = np.arange(self.rows) + 0.5
        gx, gy = np.meshgrid(cx, cy)

        # Compute received power from each tower
        signals = np.zeros((self.rows, self.cols, len(tower_positions)))
        for idx, (tx, ty, pw) in enumerate(tower_positions):
            dist = np.sqrt((gx - tx) ** 2 + (gy - ty) ** 2)
            dist = np.maximum(dist, 0.1)
            signals[:, :, idx] = pw / (dist ** PATH_LOSS_EXP)

        # SINR for each cell: best signal / (sum of others + noise)
        best_signal = np.max(signals, axis=2)
        total_signal = np.sum(signals, axis=2)
        interference = total_signal - best_signal + NOISE_FLOOR
        sinr = best_signal / interference

        # Convert to dB, clamp
        sinr_db = 10 * np.log10(np.maximum(sinr, 1e-10))
        sinr_db = np.clip(sinr_db, -10, 40)

        # Normalize to 0-1 for colormap
        sinr_norm = (sinr_db + 10) / 50.0  # -10..40 -> 0..1

        # Apply colormap: blue -> cyan -> green -> yellow -> red
        for j in range(self.rows):
            for i in range(self.cols):
                val = sinr_norm[j, i]
                alpha = 100  # semi-transparent overlay

                if val < 0.25:
                    t2 = val / 0.25
                    r, g, b = 0, int(t2 * 180), int(200 + t2 * 55)
                elif val < 0.5:
                    t2 = (val - 0.25) / 0.25
                    r, g, b = 0, int(180 + t2 * 75), int(255 - t2 * 100)
                elif val < 0.75:
                    t2 = (val - 0.5) / 0.25
                    r, g, b = int(t2 * 255), 255, int(155 - t2 * 155)
                else:
                    t2 = (val - 0.75) / 0.25
                    r, g, b = 255, int(255 - t2 * 180), 0

                color = QColor(r, g, b, alpha)
                for dy in range(cs):
                    for dx in range(cs):
                        img.setPixelColor(i * cs + dx, j * cs + dy, color)

        self._sinr_pixmap = QPixmap.fromImage(img)

    # ── Paint ──────────────────────────────────────────────────────────
    def paintEvent(self, event):
        if self._base_pixmap is None:
            self._build_base_pixmap()

        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Background
        painter.fillRect(self.rect(), QColor(18, 18, 35))

        painter.save()
        painter.translate(self._pan_offset)
        painter.scale(self._zoom, self._zoom)

        cs = self.cell_size
        w, h = self.cols * cs, self.rows * cs

        # Base grid
        painter.drawPixmap(0, 0, self._base_pixmap)

        # SINR overlay
        if self.show_sinr:
            if self._sinr_pixmap is None:
                self._build_sinr_pixmap()
            painter.drawPixmap(0, 0, self._sinr_pixmap)

        # Grid lines
        pen = QPen(QColor(255, 255, 255, 20))
        pen.setWidthF(0.5)
        painter.setPen(pen)
        for i in range(self.cols + 1):
            painter.drawLine(i * cs, 0, i * cs, h)
        for j in range(self.rows + 1):
            painter.drawLine(0, j * cs, w, j * cs)

        # Demand text
        if cs * self._zoom >= 18:
            font_size = max(7, cs // 5)
            painter.setFont(QFont("Consolas", font_size))
            for j in range(self.rows):
                for i in range(self.cols):
                    dem = self.grid_demand[j][i]
                    if dem > 0:
                        color = QColor(255, 255, 255, 180) if self.grid_buildings[j][i] else QColor(150, 170, 200, 160)
                        painter.setPen(color)
                        rect = QRectF(i * cs, j * cs, cs, cs)
                        painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, str(int(dem)))

        # Tower overlays
        self._paint_towers(painter)

        painter.restore()
        painter.end()

    def _paint_towers(self, painter):
        if not self.towers:
            return

        cs = self.cell_size
        power_map = {tt["name"]: tt["power"] for tt in self.tower_types}
        type_names = [tt["name"] for tt in self.tower_types]
        color_map = {}
        for idx, tt in enumerate(self.tower_types):
            name = tt["name"]
            color_map[name] = QColor(TOWER_COLORS.get(name, PALETTE_FALLBACK[idx % len(PALETTE_FALLBACK)]))

        marker_size = max(4, cs // 3)

        for t in self.towers:
            x, y, name = float(t["x"]), float(t["y"]), t["name"]
            px, py = (x + 0.5) * cs, (y + 0.5) * cs
            color = color_map.get(name, QColor("#999"))

            # Effective radius
            if self.show_eff_radii:
                pw = power_map.get(name, 0.01)
                er = effective_radius(pw) * cs
                pen = QPen(color, 1.5)
                painter.setPen(pen)
                painter.setBrush(QBrush(QColor(color.red(), color.green(), color.blue(), 25)))
                painter.drawEllipse(QPointF(px, py), er, er)

            # Given radius (dashed)
            if self.show_given_radii:
                pw = power_map.get(name, 0.01)
                gr = effective_radius(pw) * 0.55 * cs
                pen = QPen(color, 1.0, Qt.PenStyle.DashLine)
                painter.setPen(pen)
                painter.setBrush(Qt.BrushStyle.NoBrush)
                painter.drawEllipse(QPointF(px, py), gr, gr)

            # Marker
            type_idx = type_names.index(name) if name in type_names else 2
            painter.setPen(QPen(QColor(255, 255, 255, 200), 1))
            painter.setBrush(QBrush(color))

            if type_idx == 0:  # Triangle
                path = QPainterPath()
                path.moveTo(px, py - marker_size)
                path.lineTo(px - marker_size, py + marker_size)
                path.lineTo(px + marker_size, py + marker_size)
                path.closeSubpath()
                painter.drawPath(path)
            elif type_idx == 1:  # Square
                ms = marker_size // 2
                painter.drawRect(QRectF(px - ms, py - ms, marker_size, marker_size))
            else:  # Circle
                ms = marker_size // 2
                painter.drawEllipse(QPointF(px, py), ms, ms)

    # ── Mouse interaction ──────────────────────────────────────────────
    def _screen_to_grid(self, pos):
        """Convert screen position to grid (col, row)."""
        x = (pos.x() - self._pan_offset.x()) / self._zoom
        y = (pos.y() - self._pan_offset.y()) / self._zoom
        col = int(x / self.cell_size)
        row = int(y / self.cell_size)
        if 0 <= col < self.cols and 0 <= row < self.rows:
            return col, row
        return None, None

    def mousePressEvent(self, event: QMouseEvent):
        if event.button() == Qt.MouseButton.MiddleButton:
            self._panning = True
            self._last_pan_pos = event.position()
            self.setCursor(QCursor(Qt.CursorShape.ClosedHandCursor))
            return

        if event.button() == Qt.MouseButton.LeftButton and self.editable:
            self._apply_tool(event.position())

    def mouseMoveEvent(self, event: QMouseEvent):
        if self._panning:
            delta = event.position() - self._last_pan_pos
            self._pan_offset += delta
            self._last_pan_pos = event.position()
            self.update()
            return

        col, row = self._screen_to_grid(event.position())
        if col is not None:
            self.cell_hovered.emit(col, row)

        if event.buttons() & Qt.MouseButton.LeftButton and self.editable:
            self._apply_tool(event.position())

    def mouseReleaseEvent(self, event: QMouseEvent):
        if event.button() == Qt.MouseButton.MiddleButton:
            self._panning = False
            self.setCursor(QCursor(Qt.CursorShape.CrossCursor))

    def wheelEvent(self, event: QWheelEvent):
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        # Zoom toward cursor
        old_pos = (event.position() - self._pan_offset) / self._zoom
        self._zoom = max(0.1, min(10.0, self._zoom * factor))
        new_pos = old_pos * self._zoom + self._pan_offset
        self._pan_offset += event.position() - new_pos
        self.update()

    def _apply_tool(self, pos):
        col, row = self._screen_to_grid(pos)
        if col is None:
            return
        if self.tool == "building":
            self.grid_buildings[row][col] = not self.grid_buildings[row][col]
            self.grid_demand[row][col] = self.demand_value * 5 if self.grid_buildings[row][col] else 0
        elif self.tool == "demand":
            if not self.grid_buildings[row][col]:
                self.grid_demand[row][col] = self.demand_value
        elif self.tool == "clear":
            self.grid_buildings[row][col] = False
            self.grid_demand[row][col] = 0
        self.invalidate_base()
        self.update()


# ══════════════════════════════════════════════════════════════════════
#  TOWER TYPE EDITOR WIDGET
# ══════════════════════════════════════════════════════════════════════
class TowerTypeEditor(QWidget):
    types_changed = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.tower_types = [dict(t) for t in DEFAULT_TOWER_TYPES]
        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self._layout.setSpacing(4)
        self._entries = []
        self._rebuild()

    def _rebuild(self):
        for e in self._entries:
            e["widget"].deleteLater()
        self._entries = []

        for idx, tt in enumerate(self.tower_types):
            frame = QFrame()
            frame.setStyleSheet("QFrame { background: #16213e; border-radius: 4px; padding: 4px; }")
            fl = QGridLayout(frame)
            fl.setContentsMargins(6, 4, 6, 4)
            fl.setSpacing(4)

            name_edit = self._make_line_edit(tt["name"])
            fl.addWidget(QLabel("Name"), 0, 0)
            fl.addWidget(name_edit, 0, 1)

            color_dot = QLabel("●")
            c = TOWER_COLORS.get(tt["name"], PALETTE_FALLBACK[idx % len(PALETTE_FALLBACK)])
            color_dot.setStyleSheet(f"color: {c}; font-size: 18px;")
            fl.addWidget(color_dot, 0, 2)

            del_btn = QPushButton("✕")
            del_btn.setFixedSize(24, 24)
            del_btn.setStyleSheet("QPushButton { background: transparent; color: #e74c3c; font-size: 14px; border: none; } QPushButton:hover { color: #ff6b6b; }")
            del_btn.clicked.connect(lambda checked, i=idx: self._remove(i))
            fl.addWidget(del_btn, 0, 3)

            power_spin = QDoubleSpinBox()
            power_spin.setRange(0.001, 10000)
            power_spin.setDecimals(4)
            power_spin.setValue(tt["power"])
            power_spin.setSingleStep(0.001)
            fl.addWidget(QLabel("Power"), 1, 0)
            fl.addWidget(power_spin, 1, 1, 1, 2)

            cost_spin = QSpinBox()
            cost_spin.setRange(1, 1000000)
            cost_spin.setValue(int(tt["cost"]))
            fl.addWidget(QLabel("Cost $"), 2, 0)
            fl.addWidget(cost_spin, 2, 1, 1, 2)

            cap_spin = QSpinBox()
            cap_spin.setRange(1, 10000)
            cap_spin.setValue(int(tt["capacity"]))
            fl.addWidget(QLabel("Cap"), 3, 0)
            fl.addWidget(cap_spin, 3, 1, 1, 2)

            self._layout.addWidget(frame)
            self._entries.append({
                "widget": frame,
                "name": name_edit,
                "power": power_spin,
                "cost": cost_spin,
                "cap": cap_spin,
            })

    def _make_line_edit(self, text):
        from PyQt6.QtWidgets import QLineEdit
        le = QLineEdit(text)
        le.setMaximumWidth(100)
        return le

    def _remove(self, idx):
        if len(self.tower_types) <= 1:
            QMessageBox.warning(self, "Cannot Remove", "Need at least one tower type.")
            return
        self.tower_types.pop(idx)
        self._rebuild()
        self.types_changed.emit()

    def add_type(self):
        n = len(self.tower_types) + 1
        self.tower_types.append({"name": f"Type{n}", "power": 0.01, "cost": 1000, "capacity": 10})
        self._rebuild()
        self.types_changed.emit()

    def reset_defaults(self):
        self.tower_types = [dict(t) for t in DEFAULT_TOWER_TYPES]
        self._rebuild()
        self.types_changed.emit()

    def sync(self):
        """Read current widget values into self.tower_types."""
        result = []
        for e in self._entries:
            try:
                result.append({
                    "name": e["name"].text().strip(),
                    "power": e["power"].value(),
                    "cost": e["cost"].value(),
                    "capacity": e["cap"].value(),
                })
            except Exception:
                pass
        self.tower_types = result
        return result


# ══════════════════════════════════════════════════════════════════════
#  MAIN WINDOW
# ══════════════════════════════════════════════════════════════════════
class TowerMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Tower Placement — Grid Editor & Solver")
        self.setMinimumSize(1200, 800)
        self.resize(1400, 900)

        # State
        self.tower_data = {}
        self.results_data = {}
        self.current_algo = None
        self._solver_process = None

        self._build_ui()
        self._init_grid(10, 10)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.addWidget(splitter)

        # ── Left panel (scrollable) ──
        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        left_scroll.setMinimumWidth(280)
        left_scroll.setMaximumWidth(340)

        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(12, 12, 12, 12)
        left_layout.setSpacing(6)

        # Grid Setup
        left_layout.addWidget(section_label("GRID SETUP"))

        grid_form = QGridLayout()
        grid_form.setSpacing(6)

        grid_form.addWidget(QLabel("Rows:"), 0, 0)
        self.row_spin = QSpinBox()
        self.row_spin.setRange(3, 300)
        self.row_spin.setValue(10)
        grid_form.addWidget(self.row_spin, 0, 1)

        grid_form.addWidget(QLabel("Cols:"), 1, 0)
        self.col_spin = QSpinBox()
        self.col_spin.setRange(3, 300)
        self.col_spin.setValue(10)
        grid_form.addWidget(self.col_spin, 1, 1)

        grid_form.addWidget(QLabel("Budget $:"), 2, 0)
        self.budget_spin = QSpinBox()
        self.budget_spin.setRange(100, 10000000)
        self.budget_spin.setValue(25000)
        self.budget_spin.setSingleStep(1000)
        grid_form.addWidget(self.budget_spin, 2, 1)

        left_layout.addLayout(grid_form)

        btn_row = QHBoxLayout()
        new_btn = QPushButton("New Grid")
        new_btn.clicked.connect(self._on_new_grid)
        rand_btn = QPushButton("Randomize")
        rand_btn.clicked.connect(self._on_randomize)
        btn_row.addWidget(new_btn)
        btn_row.addWidget(rand_btn)
        left_layout.addLayout(btn_row)

        left_layout.addWidget(make_separator())

        # Tower Types
        left_layout.addWidget(section_label("TOWER TYPES"))
        self.tower_editor = TowerTypeEditor()
        left_layout.addWidget(self.tower_editor)

        tt_btns = QHBoxLayout()
        add_btn = QPushButton("+ Add Type")
        add_btn.clicked.connect(self.tower_editor.add_type)
        reset_btn = QPushButton("Reset Defaults")
        reset_btn.clicked.connect(self.tower_editor.reset_defaults)
        tt_btns.addWidget(add_btn)
        tt_btns.addWidget(reset_btn)
        left_layout.addLayout(tt_btns)

        left_layout.addWidget(make_separator())

        # Edit Tools
        left_layout.addWidget(section_label("EDIT TOOLS"))
        self.tool_group = QButtonGroup(self)
        for label, val in [("Toggle Building", "building"), ("Set Demand", "demand"), ("Clear Cell", "clear")]:
            rb = QRadioButton(label)
            rb.setChecked(val == "building")
            rb.toggled.connect(lambda checked, v=val: self._on_tool_changed(v) if checked else None)
            self.tool_group.addButton(rb)
            left_layout.addWidget(rb)

        demand_row = QHBoxLayout()
        demand_row.addWidget(QLabel("Demand value:"))
        self.demand_spin = QSpinBox()
        self.demand_spin.setRange(1, 100)
        self.demand_spin.setValue(5)
        demand_row.addWidget(self.demand_spin)
        left_layout.addLayout(demand_row)

        left_layout.addWidget(make_separator())

        # Solver
        left_layout.addWidget(section_label("SOLVER"))

        self.run_btn = QPushButton("▶  Run Solver")
        self.run_btn.setObjectName("runSolver")
        self.run_btn.clicked.connect(self._on_run_solver)
        left_layout.addWidget(self.run_btn)

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 0)  # indeterminate
        self.progress_bar.setVisible(False)
        left_layout.addWidget(self.progress_bar)

        hint = QLabel("Runs custom grid (Part 1)\n+ 300×250 grid (Part 2)")
        hint.setStyleSheet("color: #555; font-size: 10px;")
        left_layout.addWidget(hint)

        self.view_big_btn = QPushButton("View Big Grid Results")
        self.view_big_btn.setObjectName("viewBig")
        self.view_big_btn.clicked.connect(self._on_view_big)
        left_layout.addWidget(self.view_big_btn)

        self.status_label = QLabel("Ready")
        self.status_label.setObjectName("statusLabel")
        left_layout.addWidget(self.status_label)

        left_layout.addWidget(make_separator())

        # Results
        left_layout.addWidget(section_label("VIEW RESULTS"))

        self.algo_combo = QComboBox()
        self.algo_combo.currentTextChanged.connect(self._on_algo_selected)
        left_layout.addWidget(self.algo_combo)

        self.result_text = QTextEdit()
        self.result_text.setReadOnly(True)
        self.result_text.setMaximumHeight(180)
        left_layout.addWidget(self.result_text)

        self.show_eff_cb = QCheckBox("Show effective radii")
        self.show_eff_cb.setChecked(True)
        self.show_eff_cb.toggled.connect(self._on_radius_toggle)
        left_layout.addWidget(self.show_eff_cb)

        self.show_given_cb = QCheckBox("Show given radii")
        self.show_given_cb.setChecked(True)
        self.show_given_cb.toggled.connect(self._on_radius_toggle)
        left_layout.addWidget(self.show_given_cb)

        self.show_sinr_cb = QCheckBox("Show SINR heatmap")
        self.show_sinr_cb.setChecked(False)
        self.show_sinr_cb.toggled.connect(self._on_sinr_toggle)
        left_layout.addWidget(self.show_sinr_cb)

        left_layout.addWidget(make_separator())

        # Import / Export
        left_layout.addWidget(section_label("FILE"))
        export_btn = QPushButton("Export Grid CSV")
        export_btn.clicked.connect(self._on_export)
        left_layout.addWidget(export_btn)
        import_btn = QPushButton("Import Grid CSV")
        import_btn.clicked.connect(self._on_import)
        left_layout.addWidget(import_btn)

        left_layout.addStretch()

        left_scroll.setWidget(left_widget)
        splitter.addWidget(left_scroll)

        # ── Right panel: grid canvas ──
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)

        self.canvas = GridCanvas()
        self.canvas.cell_hovered.connect(self._on_cell_hovered)
        right_layout.addWidget(self.canvas)

        self.coord_label = QLabel("")
        self.coord_label.setObjectName("statusLabel")
        self.coord_label.setFixedHeight(20)
        right_layout.addWidget(self.coord_label)

        splitter.addWidget(right_widget)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)

    # ── Grid management ────────────────────────────────────────────────
    def _init_grid(self, rows, cols):
        self.tower_data = {}
        self.results_data = {}
        self.current_algo = None
        self.algo_combo.clear()
        self.result_text.clear()
        self.canvas.set_grid(rows, cols)
        self._sync_canvas_settings()

    def _sync_canvas_settings(self):
        self.canvas.demand_value = self.demand_spin.value()
        self.canvas.show_eff_radii = self.show_eff_cb.isChecked()
        self.canvas.show_given_radii = self.show_given_cb.isChecked()
        self.canvas.show_sinr = self.show_sinr_cb.isChecked()
        self.tower_editor.sync()
        self.canvas.tower_types = self.tower_editor.tower_types

    def _on_new_grid(self):
        self._init_grid(self.row_spin.value(), self.col_spin.value())
        self.status_label.setText(f"New {self.row_spin.value()}×{self.col_spin.value()} grid")

    def _on_randomize(self):
        rows, cols = self.row_spin.value(), self.col_spin.value()
        self._init_grid(rows, cols)
        n_bld = int(rows * cols * 0.15)
        for _ in range(n_bld):
            w2, h2 = random.randint(1, 2), random.randint(1, 2)
            x, y = random.randint(0, cols - w2), random.randint(0, rows - h2)
            for dy in range(h2):
                for dx in range(w2):
                    self.canvas.grid_buildings[y + dy][x + dx] = True
                    self.canvas.grid_demand[y + dy][x + dx] = random.randint(5, 50)
        for j in range(rows):
            for i in range(cols):
                if not self.canvas.grid_buildings[j][i] and random.random() < 0.4:
                    self.canvas.grid_demand[j][i] = random.randint(1, 10)
        self.canvas.invalidate_base()
        self.canvas.update()
        self.status_label.setText("Randomized")

    def _on_tool_changed(self, tool):
        self.canvas.tool = tool

    def _on_cell_hovered(self, col, row):
        bld = "Building" if self.canvas.grid_buildings[row][col] else "Open"
        dem = int(self.canvas.grid_demand[row][col])
        self.coord_label.setText(f"  ({col}, {row})  {bld}  Demand: {dem}")

    def _on_radius_toggle(self):
        self._sync_canvas_settings()
        self.canvas.update()

    def _on_sinr_toggle(self):
        self.canvas.show_sinr = self.show_sinr_cb.isChecked()
        self.canvas.invalidate_sinr()
        self.canvas.update()

    # ── Export / Import ─────────────────────────────────────────────────
    def _export_grid_csv(self, path="custom_grid.csv"):
        c = self.canvas
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["x", "y", "is_building", "demand"])
            for j in range(c.rows):
                for i in range(c.cols):
                    w.writerow([i, j, int(c.grid_buildings[j][i]), c.grid_demand[j][i]])
        return path

    def _export_tower_types_csv(self, path="tower_types.csv"):
        self.tower_editor.sync()
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["name", "power", "cost", "capacity"])
            for tt in self.tower_editor.tower_types:
                w.writerow([tt["name"], tt["power"], tt["cost"], tt["capacity"]])

    def _on_export(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export Grid CSV", "custom_grid.csv", "CSV Files (*.csv)")
        if path:
            self._export_grid_csv(path)
            self.status_label.setText(f"Exported to {os.path.basename(path)}")

    def _on_import(self):
        path, _ = QFileDialog.getOpenFileName(self, "Import Grid CSV", "", "CSV Files (*.csv)")
        if not path:
            return
        try:
            with open(path) as f:
                cells = list(csv.DictReader(f))
            max_x = max(int(c["x"]) for c in cells)
            max_y = max(int(c["y"]) for c in cells)
            rows, cols = max_y + 1, max_x + 1
            self.row_spin.setValue(rows)
            self.col_spin.setValue(cols)
            self._init_grid(rows, cols)
            for c2 in cells:
                x, y = int(c2["x"]), int(c2["y"])
                self.canvas.grid_buildings[y][x] = int(c2["is_building"]) != 0
                self.canvas.grid_demand[y][x] = float(c2["demand"])
            self.canvas.invalidate_base()
            self.canvas.update()
            self.status_label.setText(f"Imported {os.path.basename(path)}")
        except Exception as e:
            QMessageBox.critical(self, "Import Error", str(e))

    # ── Solver (async via QProcess) ────────────────────────────────────
    def _on_run_solver(self):
        if not os.path.isfile(TOWER_BINARY):
            QMessageBox.critical(
                self, "Error",
                f"Cannot find '{TOWER_BINARY}'.\n\n"
                "Compile first:\n  g++ -O2 -std=c++17 -o tower tower.cpp",
            )
            return

        self.status_label.setText("Exporting grid & tower config...")
        csv_path = self._export_grid_csv("custom_grid.csv")
        self._export_tower_types_csv("tower_types.csv")

        # Clean old results
        for f in glob.glob("small_*_towers.csv") + glob.glob("small_*.csv"):
            if os.path.exists(f):
                os.remove(f)
        for f in glob.glob("big_*_towers.csv") + glob.glob("big_*.csv"):
            if os.path.exists(f):
                os.remove(f)

        # Run via QProcess (non-blocking)
        self.run_btn.setEnabled(False)
        self.progress_bar.setVisible(True)
        self.status_label.setText("Running solver (async)...")

        self._solver_process = QProcess(self)
        self._solver_process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self._solver_process.finished.connect(self._on_solver_finished)
        self._solver_process.readyReadStandardOutput.connect(self._on_solver_output)
        self._solver_process.start(TOWER_BINARY, ["--load", csv_path, str(self.budget_spin.value())])

    def _on_solver_output(self):
        if self._solver_process:
            data = self._solver_process.readAllStandardOutput().data().decode(errors="replace")
            # Show last line as status
            lines = data.strip().split("\n")
            if lines:
                self.status_label.setText(lines[-1][:80])

    def _on_solver_finished(self, exit_code, exit_status):
        self.run_btn.setEnabled(True)
        self.progress_bar.setVisible(False)

        if exit_code != 0:
            self.status_label.setText("Solver failed")
            if self._solver_process:
                err = self._solver_process.readAllStandardOutput().data().decode(errors="replace")
                QMessageBox.critical(self, "Solver Error", err[:500])
        else:
            self._load_results()
            self.status_label.setText("Done! Select algorithm to view results.")

        self._solver_process = None

    def _load_results(self):
        self.tower_data = {}
        self.results_data = {}

        if os.path.exists("small_results.csv"):
            for row in read_csv_file("small_results.csv"):
                self.results_data[row["algorithm"]] = row

        for fpath in sorted(glob.glob("small_*_towers.csv")):
            base = os.path.basename(fpath)
            name = base.replace("small_", "").replace("_towers.csv", "")
            self.tower_data[name] = read_csv_file(fpath)

        algos = list(self.tower_data.keys())
        self.algo_combo.clear()
        self.algo_combo.addItems(algos)
        if algos:
            self.algo_combo.setCurrentText(algos[0])

    def _on_algo_selected(self, algo_name):
        if not algo_name:
            return
        self.current_algo = algo_name
        self._sync_canvas_settings()

        # Update tower overlay
        self.canvas.towers = self.tower_data.get(algo_name, [])
        self.canvas.invalidate_sinr()
        self.canvas.update()

        # Update result text
        info = self.results_data.get(algo_name, {})
        if info:
            self.result_text.setText(
                f"Algorithm: {algo_name}\n"
                f"{'─' * 30}\n"
                f"QoS Score:  {info.get('score', '?')}%\n"
                f"Towers:     {info.get('towers', '?')}\n"
                f"Cost:       ${float(info.get('cost', 0)):,.0f}\n"
                f"Nodes:      {int(float(info.get('nodes', 0))):,}\n"
                f"Time:       {info.get('time', '?')}s"
            )
        else:
            n = len(self.tower_data.get(algo_name, []))
            self.result_text.setText(f"Algorithm: {algo_name}\nTowers: {n}")

    # ── Big grid viewer ────────────────────────────────────────────────
    def _on_view_big(self):
        big_tower_files = sorted(glob.glob("big_*_towers.csv"))
        if not big_tower_files:
            QMessageBox.information(
                self, "No Data",
                "No big grid results found.\n\n"
                "Run the solver first, or run './tower' from terminal.",
            )
            return

        big_results = {}
        if os.path.exists("big_results.csv"):
            for row in read_csv_file("big_results.csv"):
                big_results[row["algorithm"]] = row

        big_towers = {}
        for fpath in big_tower_files:
            base = os.path.basename(fpath)
            name = base.replace("big_", "").replace("_towers.csv", "")
            big_towers[name] = read_csv_file(fpath)

        bb = read_csv_file("big_buildings.csv")
        bd = read_csv_file("big_demand.csv")

        R2, C2 = 250, 300
        if os.path.exists("big_grid_info.csv"):
            info = read_csv_file("big_grid_info.csv")
            if info:
                R2, C2 = int(info[0]["rows"]), int(info[0]["cols"])

        viewer = BigGridViewer(self, R2, C2, bb, bd, big_towers, big_results)
        viewer.show()


# ══════════════════════════════════════════════════════════════════════
#  BIG GRID VIEWER WINDOW
# ══════════════════════════════════════════════════════════════════════
class BigGridViewer(QMainWindow):
    def __init__(self, parent, rows, cols, buildings, demand, tower_data, results_data):
        super().__init__(parent)
        self.setWindowTitle(f"Big Grid — {cols}×{rows}")
        self.resize(1200, 800)

        self.tower_data = tower_data
        self.results_data = results_data

        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)

        # Left panel
        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setMaximumWidth(300)
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(12, 12, 12, 12)

        left_layout.addWidget(section_label(f"GRID: {cols}×{rows}"))

        self.algo_combo = QComboBox()
        self.algo_combo.addItems(list(tower_data.keys()))
        self.algo_combo.currentTextChanged.connect(self._on_algo_selected)
        left_layout.addWidget(self.algo_combo)

        self.result_text = QTextEdit()
        self.result_text.setReadOnly(True)
        self.result_text.setMaximumHeight(160)
        left_layout.addWidget(self.result_text)

        self.show_eff_cb = QCheckBox("Show effective radii")
        self.show_eff_cb.setChecked(True)
        self.show_eff_cb.toggled.connect(self._on_toggle)
        left_layout.addWidget(self.show_eff_cb)

        self.show_given_cb = QCheckBox("Show given radii")
        self.show_given_cb.setChecked(False)
        self.show_given_cb.toggled.connect(self._on_toggle)
        left_layout.addWidget(self.show_given_cb)

        self.show_sinr_cb = QCheckBox("Show SINR heatmap")
        self.show_sinr_cb.setChecked(False)
        self.show_sinr_cb.toggled.connect(self._on_toggle)
        left_layout.addWidget(self.show_sinr_cb)

        # Legend
        left_layout.addWidget(make_separator())
        left_layout.addWidget(section_label("LEGEND"))
        for name, color in TOWER_COLORS.items():
            row = QHBoxLayout()
            dot = QLabel("●")
            dot.setStyleSheet(f"color: {color}; font-size: 16px;")
            row.addWidget(dot)
            row.addWidget(QLabel(name))
            row.addStretch()
            left_layout.addLayout(row)

        left_layout.addStretch()
        left_scroll.setWidget(left_widget)
        layout.addWidget(left_scroll)

        # Canvas
        self.canvas = GridCanvas()
        self.canvas.editable = False
        cs = max(1, min(1000 // max(rows, cols), 4))
        self.canvas.cell_size = cs

        # Build grid data from CSV rows
        grid_buildings = [[False] * cols for _ in range(rows)]
        grid_demand = [[0.0] * cols for _ in range(rows)]
        for b in buildings:
            x, y = int(b["x"]), int(b["y"])
            if 0 <= y < rows and 0 <= x < cols:
                grid_buildings[y][x] = True
        for d in demand:
            x, y, val = int(d["x"]), int(d["y"]), float(d["demand"])
            if 0 <= y < rows and 0 <= x < cols:
                grid_demand[y][x] = val

        self.canvas.set_grid(rows, cols, grid_buildings, grid_demand)

        # Use big-grid powers for tower types
        big_types = []
        for tt in DEFAULT_TOWER_TYPES:
            bt = dict(tt)
            bt["power"] = POWERS_BIG.get(tt["name"], tt["power"])
            big_types.append(bt)
        self.canvas.tower_types = big_types

        layout.addWidget(self.canvas)

        # Auto-select first
        if tower_data:
            self.algo_combo.setCurrentText(list(tower_data.keys())[0])

    def _on_algo_selected(self, algo):
        if not algo:
            return
        self.canvas.towers = self.tower_data.get(algo, [])
        self._sync()
        self.canvas.invalidate_sinr()
        self.canvas.update()

        info = self.results_data.get(algo, {})
        if info:
            self.result_text.setText(
                f"Algorithm: {algo}\n"
                f"{'─' * 30}\n"
                f"QoS Score:  {info.get('score', '?')}%\n"
                f"Towers:     {info.get('towers', '?')}\n"
                f"Cost:       ${float(info.get('cost', 0)):,.0f}\n"
                f"Time:       {info.get('time', '?')}s"
            )

    def _on_toggle(self):
        self._sync()
        self.canvas.invalidate_sinr()
        self.canvas.update()

    def _sync(self):
        self.canvas.show_eff_radii = self.show_eff_cb.isChecked()
        self.canvas.show_given_radii = self.show_given_cb.isChecked()
        self.canvas.show_sinr = self.show_sinr_cb.isChecked()


# ══════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════
def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(DARK_QSS)

    # Try to set application font
    preferred_fonts = ["Segoe UI", "SF Pro Display", "Helvetica Neue", "Ubuntu"]
    for font_name in preferred_fonts:
        font = QFont(font_name, 12)
        if font.exactMatch() or font_name in QFontDatabase.families():
            app.setFont(font)
            break

    window = TowerMainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()