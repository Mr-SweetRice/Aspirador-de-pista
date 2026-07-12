from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PySide6.QtWidgets import QGridLayout, QWidget

from telemetry.state import RobotState


class PlotsPanel(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        pg.setConfigOptions(antialias=True)

        layout = QGridLayout(self)
        self.rpm_plot = pg.PlotWidget(title="RPM")
        self.battery_plot = pg.PlotWidget(title="Bateria")

        self.left_rpm_curve = self.rpm_plot.plot(pen=pg.mkPen("#2f80ed", width=2), name="left")
        self.right_rpm_curve = self.rpm_plot.plot(pen=pg.mkPen("#27ae60", width=2), name="right")
        self.battery_curve = self.battery_plot.plot(pen=pg.mkPen("#f2994a", width=2))

        for plot in [self.rpm_plot, self.battery_plot]:
            plot.showGrid(x=True, y=True, alpha=0.25)

        self.rpm_plot.setMouseEnabled(y=False)
        self.rpm_plot.setLimits(yMin=0, yMax=3000)
        self.rpm_plot.enableAutoRange(axis="y", enable=False)
        self.rpm_plot.setYRange(0, 3000, padding=0)

        layout.addWidget(self.rpm_plot, 0, 0)
        layout.addWidget(self.battery_plot, 1, 0)

    def refresh(self, state: RobotState) -> None:
        t = state.plot_time()
        if t.size == 0:
            return

        rpm = np.array(state.rpm_history)
        battery = np.array(state.battery_history)

        self.left_rpm_curve.setData(t, rpm[:, 0])
        self.right_rpm_curve.setData(t, rpm[:, 1])
        self.rpm_plot.enableAutoRange(axis="y", enable=False)
        self.rpm_plot.setYRange(0, 3000, padding=0)
        self.battery_curve.setData(t, battery)
