from __future__ import annotations

import math

import pyqtgraph as pg
from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import QBrush, QColor, QPainterPath, QPen
from PySide6.QtWidgets import QGraphicsPathItem


class GpsRobotMarker:
    def __init__(self) -> None:
        self.shadow = self._make_item(self._make_path(scale=0.56), "#06080b", "#06080b", 1.0, 0.30)
        self.outline = self._make_item(self._make_path(scale=0.54), "#f7fbff", "#f7fbff", 1.0, 0.95)
        self.arrow = self._make_item(self._make_path(scale=0.5), "#ffd15a", "#f2b233", 1.2, 1.0)

    def add_to(self, plot: pg.PlotWidget) -> None:
        plot.addItem(self.shadow)
        plot.addItem(self.outline)
        plot.addItem(self.arrow)

    def set_pose(self, x_m: float, y_m: float, heading_rad: float) -> None:
        angle_deg = 90.0 - math.degrees(heading_rad)
        self.shadow.setPos(x_m + 0.015, y_m - 0.015)
        self.outline.setPos(x_m, y_m)
        self.arrow.setPos(x_m, y_m)
        self.shadow.setRotation(angle_deg)
        self.outline.setRotation(angle_deg)
        self.arrow.setRotation(angle_deg)

    @staticmethod
    def _make_path(scale: float) -> QPainterPath:
        points = [
            QPointF(0.0, -25.0 * scale),
            QPointF(-15.0 * scale, 20.0 * scale),
            QPointF(0.0, 12.0 * scale),
            QPointF(15.0 * scale, 20.0 * scale),
        ]
        path = QPainterPath(points[0])
        for point in points[1:]:
            path.lineTo(point)
        path.closeSubpath()
        return path

    @staticmethod
    def _make_item(path: QPainterPath, fill: str, stroke: str, stroke_width: float, opacity: float) -> QGraphicsPathItem:
        item = QGraphicsPathItem(path)
        item.setBrush(QBrush(QColor(fill)))
        item.setPen(QPen(QColor(stroke), stroke_width))
        item.setOpacity(opacity)
        item.setTransformOriginPoint(0.0, 0.0)
        item.setFlag(QGraphicsPathItem.GraphicsItemFlag.ItemIgnoresTransformations, True)
        item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
        item.setZValue(20)
        return item
