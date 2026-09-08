from __future__ import annotations

import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import (
    QComboBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from ble.protocol import LINE_TRACK_BLACK, LINE_TRACK_LABELS, LINE_TRACK_WHITE
from telemetry.state import RobotState


class LinePositionView(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._position = 0
        self._visible = False
        self._values: tuple[int, ...] = (0, 0, 0, 0, 0, 0, 0, 0)
        self.setMinimumHeight(90)

    def set_line_state(self, position: int, visible: bool, values: tuple[int, ...]) -> None:
        self._position = max(0, min(7000, int(position)))
        self._visible = bool(visible)
        self._values = tuple(values[:8])
        self.update()

    def paintEvent(self, event) -> None:
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        rect = self.rect().adjusted(18, 14, -18, -18)
        track_y = rect.center().y()
        left = rect.left()
        right = rect.right()
        width = max(1, right - left)

        painter.setPen(QPen(QColor("#3a4654"), 2))
        painter.drawLine(left, track_y, right, track_y)

        for index in range(8):
            sensor_x = left + int((index / 7.0) * width)
            value = int(self._values[index]) if index < len(self._values) else 0
            value = max(0, min(1000, value))
            radius = 6 + int((value / 1000.0) * 10)
            color = QColor("#56ccf2") if value > 80 else QColor("#55616f")
            color.setAlpha(90 + int((value / 1000.0) * 165))
            painter.setBrush(color)
            painter.setPen(QPen(QColor("#1b222b"), 1))
            painter.drawEllipse(sensor_x - radius, track_y - radius, radius * 2, radius * 2)
            painter.setPen(QColor("#9fb0c3"))
            painter.drawText(sensor_x - 8, rect.bottom(), f"{index + 1}")

        marker_x = left + int((self._position / 7000.0) * width)
        marker_color = QColor("#f2c94c") if self._visible else QColor("#7b8794")
        painter.setBrush(marker_color)
        painter.setPen(QPen(QColor("#101418"), 2))
        painter.drawEllipse(marker_x - 11, track_y - 11, 22, 22)

        painter.setPen(QColor("#d6dde6"))
        painter.drawText(rect, Qt.AlignTop | Qt.AlignRight, f"{self._position} / 7000")


class LineSensorPanel(QWidget):
    calibrate_requested = Signal(int)
    track_type_requested = Signal(int)
    threshold_requested = Signal(int)
    filter_requested = Signal(int)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QScrollArea.NoFrame)
        outer.addWidget(scroll)

        content = QWidget()
        scroll.setWidget(content)

        root = QVBoxLayout(content)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        controls = QGroupBox("Sensor de linha QRE-8D")
        controls_layout = QHBoxLayout(controls)
        self.calibrate_button = QPushButton("Calibrar")
        self.calibration_time_input = QSpinBox()
        self.calibration_time_input.setRange(1, 120)
        self.calibration_time_input.setValue(5)
        self.calibration_time_input.setSuffix(" s")
        self.track_type_combo = QComboBox()
        self.track_type_combo.addItem(LINE_TRACK_LABELS[LINE_TRACK_BLACK], LINE_TRACK_BLACK)
        self.track_type_combo.addItem(LINE_TRACK_LABELS[LINE_TRACK_WHITE], LINE_TRACK_WHITE)
        self.threshold_input = QSpinBox()
        self.threshold_input.setRange(0, 45)
        self.threshold_input.setValue(0)
        self.threshold_input.setSuffix(" %")
        self.filter_slider = QSlider(Qt.Horizontal)
        self.filter_slider.setRange(0, 100)
        self.filter_slider.setValue(100)
        self.filter_slider.setMinimumWidth(150)
        self.filter_value_label = QLabel("100 %")
        self.status_label = QLabel("-")
        controls_layout.addWidget(self.calibrate_button)
        controls_layout.addWidget(QLabel("Tempo"))
        controls_layout.addWidget(self.calibration_time_input)
        controls_layout.addWidget(QLabel("Tipo de pista"))
        controls_layout.addWidget(self.track_type_combo)
        controls_layout.addWidget(QLabel("Limiar"))
        controls_layout.addWidget(self.threshold_input)
        controls_layout.addWidget(QLabel("Filtro"))
        controls_layout.addWidget(self.filter_slider)
        controls_layout.addWidget(self.filter_value_label)
        controls_layout.addWidget(self.status_label, 1)
        root.addWidget(controls)

        values = QGroupBox("Leituras")
        grid = QGridLayout(values)
        self.raw_labels: list[QLabel] = []
        self.calibrated_bars: list[QProgressBar] = []
        self.line_bars: list[QProgressBar] = []

        grid.addWidget(QLabel("Sensor"), 0, 0)
        grid.addWidget(QLabel("Raw us"), 0, 1)
        grid.addWidget(QLabel("Calibrado"), 0, 2)
        grid.addWidget(QLabel("Linha ativa"), 0, 3)

        for index in range(8):
            raw_label = QLabel("0")
            calibrated_bar = self._make_bar()
            line_bar = self._make_bar()
            self.raw_labels.append(raw_label)
            self.calibrated_bars.append(calibrated_bar)
            self.line_bars.append(line_bar)
            row = index + 1
            grid.addWidget(QLabel(f"QTR {index + 1}"), row, 0)
            grid.addWidget(raw_label, row, 1)
            grid.addWidget(calibrated_bar, row, 2)
            grid.addWidget(line_bar, row, 3)
        root.addWidget(values)

        self.position_view = LinePositionView()
        root.addWidget(self.position_view)

        summary = QGroupBox("Resultado")
        form = QFormLayout(summary)
        self.position_label = QLabel("-")
        self.visible_label = QLabel("-")
        self.calibration_label = QLabel("-")
        self.frequency_label = QLabel("-")
        form.addRow("posicao", self.position_label)
        form.addRow("linha detectada", self.visible_label)
        form.addRow("calibracao", self.calibration_label)
        form.addRow("frequencia", self.frequency_label)
        root.addWidget(summary)

        frequency_group = QGroupBox("Frequencia de leitura")
        frequency_layout = QVBoxLayout(frequency_group)
        frequency_layout.setContentsMargins(10, 16, 10, 10)
        self.frequency_plot = pg.PlotWidget()
        self.frequency_plot.setMinimumHeight(180)
        self.frequency_plot.setMouseEnabled(x=False, y=False)
        self.frequency_plot.setMenuEnabled(False)
        self.frequency_plot.hideButtons()
        self.frequency_plot.showGrid(x=True, y=True, alpha=0.25)
        self.frequency_plot.setLabel("left", "Hz")
        self.frequency_plot.setLabel("bottom", "amostra")
        self.frequency_plot.setYRange(0, 600, padding=0.0)
        self.frequency_curve = self.frequency_plot.plot(pen=pg.mkPen("#56ccf2", width=2))
        frequency_layout.addWidget(self.frequency_plot)
        root.addWidget(frequency_group)
        root.addStretch(1)

        self._hz_history: list[float] = []
        self._filter_send_timer = QTimer(self)
        self._filter_send_timer.setSingleShot(True)
        self._filter_send_timer.setInterval(120)
        self._filter_send_timer.timeout.connect(self._emit_filter)

        self.calibrate_button.clicked.connect(self._emit_calibrate)
        self.track_type_combo.currentIndexChanged.connect(self._emit_track_type)
        self.threshold_input.editingFinished.connect(self._emit_threshold)
        self.filter_slider.valueChanged.connect(self._on_filter_changed)

    def refresh(self, state: RobotState) -> None:
        for index in range(8):
            raw = int(state.line_raw[index]) if index < len(state.line_raw) else 0
            calibrated = int(state.line_calibrated[index]) if index < len(state.line_calibrated) else 0
            line_value = int(state.line_values[index]) if index < len(state.line_values) else 0
            self.raw_labels[index].setText(str(raw))
            self.calibrated_bars[index].setValue(max(0, min(1000, calibrated)))
            self.line_bars[index].setValue(max(0, min(1000, line_value)))

        if self.track_type_combo.currentData() != state.line_track_type:
            self.track_type_combo.blockSignals(True)
            for index in range(self.track_type_combo.count()):
                if self.track_type_combo.itemData(index) == state.line_track_type:
                    self.track_type_combo.setCurrentIndex(index)
                    break
            self.track_type_combo.blockSignals(False)

        self.position_label.setText(f"{state.line_position} / 7000")
        self.visible_label.setText("sim" if state.line_visible else "nao")
        self.position_view.set_line_state(state.line_position, state.line_visible, state.line_values)
        if self.threshold_input.value() != state.line_threshold_percent and not self.threshold_input.hasFocus():
            self.threshold_input.blockSignals(True)
            self.threshold_input.setValue(state.line_threshold_percent)
            self.threshold_input.blockSignals(False)
        if self.filter_slider.value() != state.line_filter_percent and not self.filter_slider.isSliderDown():
            self.filter_slider.blockSignals(True)
            self.filter_slider.setValue(state.line_filter_percent)
            self.filter_value_label.setText(f"{state.line_filter_percent} %")
            self.filter_slider.blockSignals(False)
        if state.line_calibrating:
            calibration = "calibrando"
        elif state.line_calibrated_valid:
            calibration = "calibrado"
        else:
            calibration = "pendente"
        self.calibration_label.setText(calibration)
        self.status_label.setText(LINE_TRACK_LABELS.get(state.line_track_type, str(state.line_track_type)))
        self.frequency_label.setText(f"{state.line_read_hz:.1f} Hz")
        self._hz_history.append(float(state.line_read_hz))
        self._hz_history = self._hz_history[-600:]
        self.frequency_curve.setData(list(range(len(self._hz_history))), self._hz_history)

    def _emit_track_type(self) -> None:
        data = self.track_type_combo.currentData()
        if data is not None:
            self.track_type_requested.emit(int(data))

    def _emit_calibrate(self) -> None:
        self.calibrate_requested.emit(self.calibration_time_input.value())

    def _emit_threshold(self) -> None:
        self.threshold_requested.emit(self.threshold_input.value())

    def _on_filter_changed(self, value: int) -> None:
        self.filter_value_label.setText(f"{value} %")
        self._filter_send_timer.start()

    def _emit_filter(self) -> None:
        self.filter_requested.emit(self.filter_slider.value())

    @staticmethod
    def _make_bar() -> QProgressBar:
        bar = QProgressBar()
        bar.setRange(0, 1000)
        bar.setTextVisible(True)
        return bar
