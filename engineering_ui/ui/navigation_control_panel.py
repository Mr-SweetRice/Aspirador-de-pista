from __future__ import annotations

from math import isclose, isfinite

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QLocale, Qt, QTimer, Signal
from PySide6.QtGui import QValidator
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from ble.protocol import (
    CONTROL_MODE_AUTO_TRACK,
    CONTROL_MODE_LABELS,
    CONTROL_MODE_LINE,
    CONTROL_MODE_ODOMETRY,
    ODOMETRY_SOURCE_FUSED,
)
from telemetry.state import RobotState
from ui.robot_marker import GpsRobotMarker


class PidDoubleSpinBox(QDoubleSpinBox):
    def valueFromText(self, text: str) -> float:  # noqa: N802
        return float(text.strip().replace(",", "."))

    def textFromValue(self, value: float) -> str:  # noqa: N802
        return f"{value:.{self.decimals()}f}"

    def validate(self, text: str, pos: int):  # noqa: N802
        normalized = text.strip().replace(",", ".")
        if normalized in {"", ".", ","}:
            return (QValidator.State.Intermediate, text, pos)
        try:
            value = float(normalized)
        except ValueError:
            return (QValidator.State.Invalid, text, pos)
        if self.minimum() <= value <= self.maximum():
            return (QValidator.State.Acceptable, text, pos)
        return (QValidator.State.Intermediate, text, pos)


class NavigationControlPanel(QWidget):
    MAP_MAX_SEGMENT_M = 0.30

    refresh_maps_requested = Signal()
    map_load_requested = Signal(int)
    map_first_point_reset_requested = Signal(float, float)
    start_requested = Signal(int, int, int)
    line_start_requested = Signal(int)
    auto_track_start_requested = Signal(int, int, int)
    stop_requested = Signal()
    pid_requested = Signal(float, float, float, int, float)
    pid_save_requested = Signal(float, float, float, int, int, float)
    aux_requested = Signal(int)
    battery_compensation_requested = Signal(bool)
    zero_brake_requested = Signal(bool)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._maps: list[dict] = []
        self._loaded_slot: int | None = None
        self._loaded_points: list[tuple[float, float]] = []
        self._loading_map = False
        self._loading_received = 0
        self._robot_pose_override: tuple[float, float, float] | None = None
        self._updating_maps = False
        self._pid_dirty = False
        self._pending_pid: tuple[float, float, float, int, float] | None = None
        self._aux_dirty = False
        self._pending_aux: int | None = None
        self._error_history: list[float] = []
        self._pid_timer = QTimer(self)
        self._pid_timer.setSingleShot(True)
        self._pid_timer.setInterval(150)
        self._pid_timer.timeout.connect(self._emit_pid_live)
        self._aux_timer = QTimer(self)
        self._aux_timer.setSingleShot(True)
        self._aux_timer.setInterval(150)
        self._aux_timer.timeout.connect(self._emit_aux_live)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)

        side_panel = QWidget()
        side_panel.setMinimumWidth(360)
        side_layout = QVBoxLayout(side_panel)
        side_layout.setContentsMargins(0, 0, 0, 0)
        side_layout.setSpacing(10)

        command_group = QGroupBox("Controle")
        command_layout = QFormLayout(command_group)
        command_layout.setContentsMargins(16, 20, 16, 14)
        command_layout.setVerticalSpacing(10)

        self.map_combo = QComboBox()
        self.mode_combo = QComboBox()
        self.mode_combo.addItem(CONTROL_MODE_LABELS[CONTROL_MODE_LINE], CONTROL_MODE_LINE)
        self.mode_combo.addItem(CONTROL_MODE_LABELS[CONTROL_MODE_AUTO_TRACK], CONTROL_MODE_AUTO_TRACK)
        self.mode_combo.addItem(CONTROL_MODE_LABELS[CONTROL_MODE_ODOMETRY], CONTROL_MODE_ODOMETRY)
        self.speed_input = QSpinBox()
        self.speed_input.setRange(-100, 100)
        self.speed_input.setValue(25)
        self.speed_input.setSuffix(" %")
        self.aux_input = QSpinBox()
        self.aux_input.setRange(0, 100)
        self.aux_input.setValue(0)
        self.aux_input.setSuffix(" %")
        self.kp_input = PidDoubleSpinBox()
        self._configure_pid_input(self.kp_input, 0.1)
        self.kp_input.setValue(35.0)
        self.ki_input = PidDoubleSpinBox()
        self._configure_pid_input(self.ki_input, 0.01)
        self.ki_input.setValue(0.0)
        self.kd_input = PidDoubleSpinBox()
        self._configure_pid_input(self.kd_input, 0.01)
        self.kd_input.setValue(0.0)
        self.alpha_input = PidDoubleSpinBox()
        self._configure_pid_input(self.alpha_input, 0.01, minimum=0.0, maximum=1.0)
        self.alpha_input.setValue(0.7)
        self.motor_limit_input = QSpinBox()
        self.motor_limit_input.setRange(0, 100)
        self.motor_limit_input.setValue(100)
        self.motor_limit_input.setSuffix(" %")
        self.apply_pid_button = QPushButton("Salvar PID/limite/turbina")
        self.battery_compensation_check = QCheckBox("Compensar pela tensao da bateria")
        self.battery_compensation_check.setToolTip("Corrige PWM usando tensao atual da bateria contra 12,6 V")
        self.zero_brake_check = QCheckBox("Freio em 0%")
        self.zero_brake_check.setChecked(True)
        self.zero_brake_check.setToolTip("Quando ligado, comando 0% usa short brake no TB6612; desligado deixa o motor livre")

        buttons = QWidget()
        buttons_layout = QHBoxLayout(buttons)
        buttons_layout.setContentsMargins(0, 0, 0, 0)
        self.start_button = QPushButton("Start")
        self.stop_button = QPushButton("Stop controle")
        self.stop_button.setObjectName("stopButton")
        self.stop_button.setToolTip("Para o controle e corta os motores imediatamente")
        buttons_layout.addWidget(self.start_button)
        buttons_layout.addWidget(self.stop_button)

        command_layout.addRow("Modo", self.mode_combo)
        command_layout.addRow("Velocidade", self.speed_input)
        command_layout.addRow("Turbina", self.aux_input)
        command_layout.addRow("Kp", self.kp_input)
        command_layout.addRow("Ki", self.ki_input)
        command_layout.addRow("Kd", self.kd_input)
        command_layout.addRow("Alpha P", self.alpha_input)
        command_layout.addRow("Limite motor", self.motor_limit_input)
        command_layout.addRow(self.battery_compensation_check)
        command_layout.addRow(self.zero_brake_check)
        command_layout.addRow(self.apply_pid_button)
        command_layout.addRow(buttons)
        side_layout.addWidget(command_group)

        status_group = QGroupBox("Estado")
        status_layout = QFormLayout(status_group)
        status_layout.setContentsMargins(16, 20, 16, 14)
        self.running_label = QLabel("-")
        self.target_label = QLabel("-")
        self.error_label = QLabel("-")
        self.distance_label = QLabel("-")
        self.target_xy_label = QLabel("-")
        self.steer_label = QLabel("-")
        self.pid_label = QLabel("-")
        self.limit_label = QLabel("-")
        self.active_speed_label = QLabel("-")
        self.aux_label = QLabel("-")
        self.average_speed_label = QLabel("-")
        self.max_speed_label = QLabel("-")
        status_layout.addRow("rodando", self.running_label)
        status_layout.addRow("target", self.target_label)
        status_layout.addRow("erro angulo", self.error_label)
        status_layout.addRow("distancia", self.distance_label)
        status_layout.addRow("ponto", self.target_xy_label)
        status_layout.addRow("correcao", self.steer_label)
        status_layout.addRow("pid atual", self.pid_label)
        status_layout.addRow("limite motor", self.limit_label)
        status_layout.addRow("vel ativa", self.active_speed_label)
        status_layout.addRow("turbina", self.aux_label)
        status_layout.addRow("vel media", self.average_speed_label)
        status_layout.addRow("vel maxima", self.max_speed_label)
        side_layout.addWidget(status_group)

        side_layout.addStretch(1)

        side_scroll = QScrollArea()
        side_scroll.setWidgetResizable(True)
        side_scroll.setFrameShape(QScrollArea.NoFrame)
        side_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        side_scroll.setWidget(side_panel)
        side_scroll.setMaximumWidth(460)
        side_scroll.setMinimumWidth(380)
        layout.addWidget(side_scroll, 0)

        self.map_group = QGroupBox("Mapa da pista")
        map_layout = QVBoxLayout(self.map_group)
        map_layout.setContentsMargins(8, 14, 8, 8)
        map_header = QWidget()
        map_header_layout = QHBoxLayout(map_header)
        map_header_layout.setContentsMargins(0, 0, 0, 0)
        self.refresh_button = QPushButton("Atualizar")
        self.refresh_button.setToolTip("Atualizar mapas salvos")
        self.reset_to_first_point_button = QPushButton("Reset posicao")
        self.reset_to_first_point_button.setToolTip("Resetar posicao e angulo para o primeiro ponto do mapa")
        self.auto_reset_first_point_check = QCheckBox("Auto reset posicao")
        self.auto_reset_first_point_check.setChecked(True)
        self.auto_reset_first_point_check.setToolTip("Resetar posicao e angulo para o primeiro ponto antes de qualquer Start")
        self.map_combo.setMinimumWidth(220)
        map_header_layout.addWidget(QLabel("Mapa"))
        map_header_layout.addWidget(self.map_combo, 2)
        map_header_layout.addWidget(self.refresh_button)
        map_header_layout.addWidget(self.reset_to_first_point_button)
        map_header_layout.addWidget(self.auto_reset_first_point_check)
        map_header_layout.addStretch(1)
        self.view_combo = QComboBox()
        self.view_combo.addItem("Mapa", "map")
        self.view_combo.addItem("Erro", "error")
        self.map_toggle_button = QPushButton("Minimizar")
        self.map_toggle_button.clicked.connect(lambda: self._set_map_collapsed(not self.map_plot.isHidden()))
        self.view_combo.currentIndexChanged.connect(self._refresh_view_mode)
        map_header_layout.addWidget(self.view_combo)
        map_header_layout.addWidget(self.map_toggle_button)
        map_layout.addWidget(map_header)
        self.map_load_progress = QProgressBar()
        self.map_load_progress.setRange(0, 100)
        self.map_load_progress.setValue(0)
        self.map_load_progress.setTextVisible(True)
        self.map_load_progress.hide()
        map_layout.addWidget(self.map_load_progress)
        self.map_plot = pg.PlotWidget()
        self.map_plot.setMinimumSize(360, 300)
        self.map_plot.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.map_plot.setAspectLocked(True)
        self.map_plot.setMouseEnabled(x=False, y=False)
        self.map_plot.setMenuEnabled(False)
        self.map_plot.hideButtons()
        self.map_plot.showGrid(x=True, y=True, alpha=0.25)
        self.track_path = self.map_plot.plot(pen=pg.mkPen("#f2c94c", width=2))
        self.target_item = pg.ScatterPlotItem(size=9, brush="#56ccf2", pen=pg.mkPen("#111820", width=1))
        self.map_plot.addItem(self.target_item)
        self.robot_marker = GpsRobotMarker()
        self.robot_marker.add_to(self.map_plot)
        self.error_plot = pg.PlotWidget()
        self.error_plot.setMinimumSize(360, 300)
        self.error_plot.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.error_plot.setMouseEnabled(x=False, y=False)
        self.error_plot.setMenuEnabled(False)
        self.error_plot.hideButtons()
        self.error_plot.showGrid(x=True, y=True, alpha=0.25)
        self.error_plot.setLabel("left", "erro")
        self.error_plot.setLabel("bottom", "amostra")
        self.error_setpoint = pg.InfiniteLine(
            pos=0.0,
            angle=0,
            pen=pg.mkPen("#56ccf2", width=1, style=Qt.PenStyle.DashLine),
        )
        self.error_plot.addItem(self.error_setpoint)
        self.error_curve = self.error_plot.plot(pen=pg.mkPen("#eb5757", width=2))
        map_layout.addWidget(self.map_plot)
        map_layout.addWidget(self.error_plot)
        layout.addWidget(self.map_group, 1)
        self._refresh_view_mode()

        self.refresh_button.clicked.connect(self.refresh_maps_requested)
        self.reset_to_first_point_button.clicked.connect(self._emit_reset_to_first_point)
        self.start_button.clicked.connect(self._emit_start)
        self.stop_button.clicked.connect(self.stop_requested)
        self.map_combo.currentIndexChanged.connect(self._emit_map_load)
        self.mode_combo.currentIndexChanged.connect(self._refresh_mode_visibility)
        self.apply_pid_button.clicked.connect(self._emit_pid_save)
        self.battery_compensation_check.toggled.connect(self.battery_compensation_requested)
        self.zero_brake_check.toggled.connect(self.zero_brake_requested)
        self.kp_input.valueChanged.connect(self._schedule_pid_live)
        self.ki_input.valueChanged.connect(self._schedule_pid_live)
        self.kd_input.valueChanged.connect(self._schedule_pid_live)
        self.alpha_input.valueChanged.connect(self._schedule_pid_live)
        self.motor_limit_input.valueChanged.connect(self._schedule_pid_live)
        for pid_input in (self.kp_input, self.ki_input, self.kd_input, self.alpha_input, self.motor_limit_input):
            pid_input.lineEdit().textEdited.connect(self._mark_pid_editing)
            pid_input.editingFinished.connect(self._schedule_pid_live)
        self.aux_input.valueChanged.connect(self._schedule_aux_live)
        self._refresh_mode_visibility()

    def set_available_maps(self, maps: list[dict]) -> None:
        self._updating_maps = True
        current_data = self.map_combo.currentData()
        current_slot = current_data.get("slot") if current_data else None
        self._maps = maps
        self.map_combo.clear()
        for item in maps:
            distance = float(item.get("distance_m", 0.0))
            self.map_combo.addItem(f"{item['name']} ({item['point_count']} pts, {distance:.2f} m)", item)
        if current_slot is not None:
            for index in range(self.map_combo.count()):
                data = self.map_combo.itemData(index)
                if data and data.get("slot") == current_slot:
                    self.map_combo.setCurrentIndex(index)
                    break
        self._updating_maps = False
        self._emit_map_load()

    def start_loaded_map(self, slot: int, total_points: int) -> None:
        self._loaded_slot = slot
        self._loaded_points = [(0.0, 0.0)] * total_points
        self._loading_map = True
        self._loading_received = 0
        self.track_path.clear()
        self.target_item.clear()
        self.map_load_progress.setRange(0, max(1, total_points))
        self.map_load_progress.setValue(0)
        self.map_load_progress.setFormat("Carregando mapa: 0%")
        self.map_load_progress.show()

    def set_loaded_chunk(self, offset: int, points: list[tuple[float, float]]) -> None:
        for index, point in enumerate(points):
            target = offset + index
            if 0 <= target < len(self._loaded_points):
                self._loaded_points[target] = point
        self._loading_received = min(len(self._loaded_points), max(self._loading_received, offset + len(points)))
        if self._loading_map:
            self.map_load_progress.setValue(self._loading_received)
            total = max(1, len(self._loaded_points))
            percent = int((self._loading_received * 100) / total)
            self.map_load_progress.setFormat(f"Carregando mapa: {percent}%")
            return
        self._draw_map()

    def finish_loaded_map(self, slot: int) -> None:
        if self._loaded_slot != slot:
            return
        self._loading_map = False
        self.map_load_progress.hide()
        self._draw_map()

    def refresh(self, state: RobotState) -> None:
        actual_pid = (
            state.control_kp,
            state.control_ki,
            state.control_kd,
            state.control_motor_limit_percent,
            state.control_line_alpha,
        )
        if self._pending_pid and self._pid_matches(self._pending_pid, actual_pid):
            self._pending_pid = None
            self._pid_dirty = False
            self.apply_pid_button.setText("Salvar PID/limite/turbina")
        if self._pending_aux is not None and int(self._pending_aux) == int(state.control_aux_percent):
            self._pending_aux = None
            self._aux_dirty = False

        if not self._pid_dirty and self._pending_pid is None:
            self._set_pid_inputs(actual_pid)
        if not self._aux_dirty and self._pending_aux is None:
            self._set_aux_input(state.control_aux_percent)
        if self.battery_compensation_check.isChecked() != state.control_battery_compensation_enabled:
            self.battery_compensation_check.blockSignals(True)
            self.battery_compensation_check.setChecked(state.control_battery_compensation_enabled)
            self.battery_compensation_check.blockSignals(False)
        self.running_label.setText("sim" if state.control_running else "nao")
        mode_label = CONTROL_MODE_LABELS.get(state.control_mode, str(state.control_mode))
        battery_label = "bat on" if state.control_battery_compensation_enabled else "bat off"
        if state.control_mode in (CONTROL_MODE_ODOMETRY, CONTROL_MODE_AUTO_TRACK) and state.control_point_count > 1:
            target_number = max(0, state.control_target_index)
            target_total = state.control_point_count - 1
            self.target_label.setText(f"{target_number}/{target_total}")
        else:
            self.target_label.setText(f"{state.control_target_index}/{state.control_point_count}")
        self.error_label.setText(f"{state.control_angle_error_rad:.3f} rad")
        self.distance_label.setText(f"{state.control_distance_m:.3f} m")
        self.target_xy_label.setText(f"{state.control_target_x_m:.3f}, {state.control_target_y_m:.3f}")
        self.steer_label.setText(f"{state.control_steer_percent:.1f} %")
        self.pid_label.setText(
            f"{state.control_kp:.3f}, {state.control_ki:.3f}, {state.control_kd:.3f} | alpha {state.control_line_alpha:.3f}"
        )
        self.limit_label.setText(f"{state.control_motor_limit_percent} % | {mode_label} | {battery_label}")
        self.active_speed_label.setText(f"{state.control_active_speed_percent} % | alvo {state.control_speed_percent} %")
        self.aux_label.setText(f"{state.control_aux_percent} % | ativo {state.control_active_aux_percent} %")
        self.average_speed_label.setText(f"{state.control_average_speed_mps:.3f} m/s")
        self.max_speed_label.setText(f"{state.control_max_speed_mps:.3f} m/s")
        if state.control_running:
            self._robot_pose_override = None
        self._draw_robot(state)
        self._draw_target(state)
        self._draw_error(state)

    def set_zero_brake_enabled(self, enabled: bool) -> None:
        self.zero_brake_check.blockSignals(True)
        self.zero_brake_check.setChecked(bool(enabled))
        self.zero_brake_check.blockSignals(False)

    def cancel_pending_commands(self) -> None:
        self._pid_timer.stop()
        self._aux_timer.stop()

    def _emit_start(self) -> None:
        mode = int(self.mode_combo.currentData())
        if mode == CONTROL_MODE_LINE:
            self._emit_auto_reset_to_first_point()
            self.line_start_requested.emit(int(self.speed_input.value()))
            return
        data = self.map_combo.currentData()
        if not data:
            return
        self._emit_auto_reset_to_first_point()
        slot = int(data["slot"])
        speed = int(self.speed_input.value())
        if mode == CONTROL_MODE_AUTO_TRACK:
            self.auto_track_start_requested.emit(slot, speed, ODOMETRY_SOURCE_FUSED)
        else:
            self.start_requested.emit(slot, speed, ODOMETRY_SOURCE_FUSED)

    def _emit_map_load(self) -> None:
        if self._updating_maps:
            return
        data = self.map_combo.currentData()
        if data:
            slot = int(data["slot"])
            if slot != self._loaded_slot:
                self.map_load_requested.emit(slot)

    def _emit_reset_to_first_point(self) -> bool:
        if self._set_robot_at_first_point() is None:
            return False
        x_m, y_m = self._loaded_points[0]
        self.map_first_point_reset_requested.emit(float(x_m), float(y_m))
        return True

    def _emit_auto_reset_to_first_point(self) -> bool:
        if self.auto_reset_first_point_check.isChecked():
            return self._emit_reset_to_first_point()
        return False

    def _refresh_mode_visibility(self) -> None:
        return

    def _set_map_collapsed(self, collapsed: bool) -> None:
        self.map_plot.setVisible(not collapsed and self.view_combo.currentData() == "map")
        self.error_plot.setVisible(not collapsed and self.view_combo.currentData() == "error")
        self.view_combo.setVisible(not collapsed)
        self.map_toggle_button.setText("Mostrar" if collapsed else "Minimizar")
        self.map_group.setMaximumWidth(180 if collapsed else 16777215)

    def _refresh_view_mode(self) -> None:
        collapsed = self.map_plot.isHidden() and self.error_plot.isHidden()
        self.map_plot.setVisible(not collapsed and self.view_combo.currentData() == "map")
        self.error_plot.setVisible(not collapsed and self.view_combo.currentData() == "error")

    def _current_pid(self) -> tuple[float, float, float, int, float]:
        self._interpret_pid_inputs()
        return (
            float(self.kp_input.value()),
            float(self.ki_input.value()),
            float(self.kd_input.value()),
            int(self.motor_limit_input.value()),
            float(self.alpha_input.value()),
        )

    def _emit_pid_live(self) -> None:
        pid = self._current_pid()
        self._pending_pid = pid
        self._pid_dirty = False
        self.pid_requested.emit(*pid)

    def _emit_pid_save(self) -> None:
        self._pid_timer.stop()
        self._interpret_pid_inputs()
        pid = (
            float(self.kp_input.value()),
            float(self.ki_input.value()),
            float(self.kd_input.value()),
            int(self.motor_limit_input.value()),
            float(self.alpha_input.value()),
        )
        self._pending_pid = pid
        self._pid_dirty = False
        self.apply_pid_button.setText("Salvando...")
        aux = int(self.aux_input.value())
        self._pending_aux = aux
        self._aux_dirty = False
        self.pid_save_requested.emit(pid[0], pid[1], pid[2], pid[3], aux, pid[4])

    def finish_pid_save(self, success: bool) -> None:
        if not success:
            self._pending_pid = None
        self._pending_aux = None
        self._pid_dirty = not success
        self._aux_dirty = False
        self.apply_pid_button.setText("PID salvo" if success else "Falha ao salvar PID")
        QTimer.singleShot(1200, lambda: self.apply_pid_button.setText("Salvar PID/limite/turbina"))

    def _schedule_pid_live(self) -> None:
        self._pid_dirty = True
        self._pending_pid = None
        self.apply_pid_button.setText("Salvar PID/limite/turbina")
        self._pid_timer.start()

    def _mark_pid_editing(self, *_args) -> None:
        self._pid_dirty = True
        self._pending_pid = None
        self.apply_pid_button.setText("Salvar PID/limite/turbina")

    def _interpret_pid_inputs(self) -> None:
        for widget in (self.kp_input, self.ki_input, self.kd_input, self.alpha_input, self.motor_limit_input):
            widget.interpretText()

    def _set_pid_inputs(self, pid: tuple[float, float, float, int, float]) -> None:
        widgets = [
            (self.kp_input, pid[0]),
            (self.ki_input, pid[1]),
            (self.kd_input, pid[2]),
            (self.motor_limit_input, pid[3]),
            (self.alpha_input, pid[4]),
        ]
        for widget, value in widgets:
            widget.blockSignals(True)
            widget.setValue(value)
            widget.blockSignals(False)

    def _schedule_aux_live(self) -> None:
        self._aux_dirty = True
        self._pending_aux = None
        self._aux_timer.start()

    def _emit_aux_live(self) -> None:
        aux = int(self.aux_input.value())
        self._pending_aux = aux
        self._aux_dirty = False
        self.aux_requested.emit(aux)

    def _set_aux_input(self, aux_percent: int) -> None:
        self.aux_input.blockSignals(True)
        self.aux_input.setValue(int(aux_percent))
        self.aux_input.blockSignals(False)

    @staticmethod
    def _pid_matches(a: tuple[float, float, float, int, float], b: tuple[float, float, float, int, float]) -> bool:
        return (
            isclose(a[0], b[0], abs_tol=0.002)
            and isclose(a[1], b[1], abs_tol=0.002)
            and isclose(a[2], b[2], abs_tol=0.002)
            and int(a[3]) == int(b[3])
            and isclose(a[4], b[4], abs_tol=0.002)
        )

    @staticmethod
    def _configure_pid_input(widget: QDoubleSpinBox, step: float, minimum: float = 0.0, maximum: float = 1000.0) -> None:
        widget.setRange(minimum, maximum)
        widget.setDecimals(3)
        widget.setSingleStep(step)
        widget.setLocale(QLocale.c())
        widget.setKeyboardTracking(False)

    def _draw_map(self) -> None:
        if not self._loaded_points or self._loading_map:
            self.track_path.clear()
            self.target_item.clear()
            return
        points = self._polyline_array(self._loaded_points, self.MAP_MAX_SEGMENT_M)
        self.track_path.setData(points[:, 0], points[:, 1])
        self.target_item.setData([points[-1, 0]], [points[-1, 1]])
        self._fit_map()

    def _draw_robot(self, state: RobotState) -> None:
        if self._robot_pose_override is not None and not state.control_running:
            x_m, y_m, heading_rad = self._robot_pose_override
            self.robot_marker.set_pose(x_m, y_m, heading_rad)
            return
        self.robot_marker.set_pose(state.x_m, state.y_m, state.heading_rad)

    def _draw_target(self, state: RobotState) -> None:
        if not self._loaded_points:
            self.target_item.clear()
            return
        if state.control_mode in (CONTROL_MODE_ODOMETRY, CONTROL_MODE_AUTO_TRACK) and state.control_running:
            self.target_item.setData([state.control_target_x_m], [state.control_target_y_m])
            return
        point = self._loaded_points[-1]
        self.target_item.setData([point[0]], [point[1]])

    def _set_robot_at_first_point(self) -> tuple[float, float, float] | None:
        if not self._loaded_points or self._loading_map:
            return None
        x_m, y_m = self._loaded_points[0]
        heading_rad = 0.0
        if len(self._loaded_points) > 1:
            next_x, next_y = self._loaded_points[1]
            heading_rad = float(np.arctan2(next_y - y_m, next_x - x_m))
        self._robot_pose_override = (float(x_m), float(y_m), heading_rad)
        self.robot_marker.set_pose(float(x_m), float(y_m), heading_rad)
        return self._robot_pose_override

    def _draw_error(self, state: RobotState) -> None:
        self._error_history.append(state.control_angle_error_rad)
        self._error_history = self._error_history[-600:]
        if self.error_plot.isVisible() and self._error_history:
            y = np.array(self._error_history)
            x = np.arange(y.size)
            self.error_curve.setData(x, y)

    def _fit_map(self) -> None:
        points: list[tuple[float, float]] = list(self._loaded_points)
        if not points:
            return
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        width = max(max(xs) - min(xs), 0.2)
        height = max(max(ys) - min(ys), 0.2)
        pad_x = width * 0.12
        pad_y = height * 0.12
        self.map_plot.setRange(
            xRange=(min(xs) - pad_x, max(xs) + pad_x),
            yRange=(min(ys) - pad_y, max(ys) + pad_y),
            padding=0.0,
        )

    @staticmethod
    def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return float((dx * dx + dy * dy) ** 0.5)

    @classmethod
    def _polyline_array(cls, points: list[tuple[float, float]], max_segment_m: float) -> np.ndarray:
        if not points:
            return np.empty((0, 2))

        output: list[tuple[float, float]] = []
        previous: tuple[float, float] | None = None
        for raw_x, raw_y in points:
            point = (float(raw_x), float(raw_y))
            if not (isfinite(point[0]) and isfinite(point[1])):
                previous = None
                if output and not np.isnan(output[-1][0]):
                    output.append((np.nan, np.nan))
                continue
            if previous is not None and cls._distance(previous, point) > max_segment_m:
                output.append((np.nan, np.nan))
            output.append(point)
            previous = point
        return np.array(output)
