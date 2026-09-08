from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import struct

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QLocale, QSettings, Qt, Signal
from PySide6.QtGui import QValidator
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QProgressBar,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QStyledItemDelegate,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ble.protocol import (
    RACE_PLAN_MAX_SEGMENTS,
    RACE_SEGMENT_CURVE,
    RACE_SEGMENT_INTERSECTION,
    RACE_SEGMENT_LABELS,
    RACE_SEGMENT_NORMAL,
    RACE_SEGMENT_STOP,
)
from telemetry.state import RobotState
from ui.robot_marker import GpsRobotMarker


SEGMENT_COLORS = {
    RACE_SEGMENT_NORMAL: "#46ff00",
    RACE_SEGMENT_INTERSECTION: "#1b0fff",
    RACE_SEGMENT_CURVE: "#ff3600",
    RACE_SEGMENT_STOP: "#eb5757",
}

AUTO_DETECT_WINDOW_POINTS = 4
AUTO_DETECT_PADDING_POINTS = 3
AUTO_DETECT_MIN_RUN_POINTS = 4
DEFAULT_PID = (35.0, 0.0, 0.0)
PROFILE_ROWS = [
    (RACE_SEGMENT_NORMAL, "Reta"),
    (RACE_SEGMENT_INTERSECTION, "Intersecao"),
    (RACE_SEGMENT_CURVE, "Curva"),
    (RACE_SEGMENT_STOP, "Parada"),
]
PROFILE_DEFAULTS = {
    RACE_SEGMENT_NORMAL: (70, 100, 0, *DEFAULT_PID),
    RACE_SEGMENT_INTERSECTION: (55, 100, 0, *DEFAULT_PID),
    RACE_SEGMENT_CURVE: (40, 100, 0, *DEFAULT_PID),
    RACE_SEGMENT_STOP: (0, 0, 100, *DEFAULT_PID),
}
PID_MIN_GAIN = 0.0
PID_MAX_GAIN = 1000.0
PID_DECIMALS = 3
RACE_PLAN_FILE_FORMAT = "aspirador-de-pista-race-plan"
RACE_PLAN_FILE_VERSION = 1


def _parse_decimal(text: str) -> float:
    return float(str(text).strip().replace(",", "."))


def _require_json_int(value: object, field: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} deve ser inteiro")
    number = float(value)
    if not math.isfinite(number) or not number.is_integer():
        raise ValueError(f"{field} deve ser inteiro")
    result = int(number)
    if not minimum <= result <= maximum:
        raise ValueError(f"{field} fora da faixa {minimum}..{maximum}")
    return result


def _require_json_float(value: object, field: str, minimum: float, maximum: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} deve ser numerico")
    result = float(value)
    if not math.isfinite(result) or not minimum <= result <= maximum:
        raise ValueError(f"{field} fora da faixa {minimum:g}..{maximum:g}")
    return result


def _require_json_bool(value: object, field: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{field} deve ser booleano")
    return value


class PidDoubleSpinBox(QDoubleSpinBox):
    def valueFromText(self, text: str) -> float:  # noqa: N802
        return _parse_decimal(text)

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


class PidGainDelegate(QStyledItemDelegate):
    def createEditor(self, parent, _option, _index):  # noqa: N802
        editor = PidDoubleSpinBox(parent)
        editor.setRange(PID_MIN_GAIN, PID_MAX_GAIN)
        editor.setDecimals(PID_DECIMALS)
        editor.setSingleStep(0.01)
        editor.setLocale(QLocale.c())
        editor.setKeyboardTracking(False)
        return editor

    def setEditorData(self, editor, index):  # noqa: N802
        if not isinstance(editor, QDoubleSpinBox):
            return super().setEditorData(editor, index)
        try:
            value = _parse_decimal(index.data())
        except (TypeError, ValueError):
            value = 0.0
        editor.setValue(max(PID_MIN_GAIN, min(PID_MAX_GAIN, value)))

    def setModelData(self, editor, model, index):  # noqa: N802
        if not isinstance(editor, QDoubleSpinBox):
            return super().setModelData(editor, model, index)
        model.setData(index, f"{editor.value():.{PID_DECIMALS}f}")


class RacePlanPanel(QWidget):
    refresh_maps_requested = Signal()
    map_load_requested = Signal(int)
    apply_requested = Signal(dict, list, bool)
    start_requested = Signal(int, int, dict, list, bool)
    stop_requested = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.settings = QSettings("AspiradorDePista", "EngineeringUI")
        self._maps: list[dict] = []
        self._loaded_slot: int | None = None
        self._loaded_points: list[tuple[float, float]] = []
        self._loading_map = False
        self._loading_received = 0
        self._updating_maps = False
        self._updating_table = False
        self._robot_pose_override: tuple[float, float, float] | None = None
        self._segment_curves: list[pg.PlotDataItem] = []

        root = QHBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        panel = QWidget()
        panel_layout = QVBoxLayout(panel)
        panel_layout.setContentsMargins(0, 0, 0, 0)
        panel_layout.setSpacing(10)

        config_group = QGroupBox("Plano de corrida")
        config_layout = QFormLayout(config_group)
        config_layout.setContentsMargins(16, 20, 16, 14)
        config_layout.setVerticalSpacing(10)

        map_row = QWidget()
        map_row_layout = QHBoxLayout(map_row)
        map_row_layout.setContentsMargins(0, 0, 0, 0)
        self.map_combo = QComboBox()
        self.map_combo.setMinimumWidth(240)
        self.refresh_button = QPushButton("Atualizar")
        map_row_layout.addWidget(self.map_combo, 1)
        map_row_layout.addWidget(self.refresh_button)

        self.start_speed_input = QSpinBox()
        self.start_speed_input.setRange(1, 100)
        self.start_speed_input.setValue(50)
        self.start_speed_input.setSuffix(" %")
        self.line_loss_odometry_check = QCheckBox("Usar odometria se perder linha")
        self.line_loss_odometry_check.setChecked(True)
        self.battery_compensation_check = QCheckBox("Compensar tensao bateria")
        self.auto_detect_check = QCheckBox("Detectar retas/curvas ao carregar")
        self.curve_threshold_input = QSpinBox()
        self.curve_threshold_input.setRange(2, 90)
        self.curve_threshold_input.setValue(10)
        self.curve_threshold_input.setSuffix(" deg")
        self.transition_extension_input = QSpinBox()
        self.transition_extension_input.setRange(0, 40)
        self.transition_extension_input.setValue(4)
        self.transition_extension_input.setSuffix(" pts/lado")

        config_layout.addRow("Mapa", map_row)
        config_layout.addRow("Vel partida", self.start_speed_input)
        config_layout.addRow("Limiar curva", self.curve_threshold_input)
        config_layout.addRow("Ext intersecao", self.transition_extension_input)
        config_layout.addRow(self.line_loss_odometry_check)
        config_layout.addRow(self.battery_compensation_check)
        config_layout.addRow(self.auto_detect_check)

        buttons = QWidget()
        button_layout = QHBoxLayout(buttons)
        button_layout.setContentsMargins(0, 0, 0, 0)
        self.apply_button = QPushButton("Enviar ao robo")
        self.start_button = QPushButton("Enviar e iniciar")
        self.stop_button = QPushButton("Stop controle")
        self.stop_button.setObjectName("stopButton")
        button_layout.addWidget(self.apply_button)
        button_layout.addWidget(self.start_button)
        button_layout.addWidget(self.stop_button)
        config_layout.addRow(buttons)

        file_buttons = QWidget()
        file_button_layout = QHBoxLayout(file_buttons)
        file_button_layout.setContentsMargins(0, 0, 0, 0)
        self.save_plan_file_button = QPushButton("Salvar JSON")
        self.load_plan_file_button = QPushButton("Carregar JSON")
        file_button_layout.addWidget(self.save_plan_file_button)
        file_button_layout.addWidget(self.load_plan_file_button)
        config_layout.addRow("Arquivo", file_buttons)
        panel_layout.addWidget(config_group)

        profile_group = QGroupBox("Padroes do plano")
        profile_layout = QVBoxLayout(profile_group)
        profile_layout.setContentsMargins(10, 16, 10, 10)
        self.profile_table = QTableWidget(len(PROFILE_ROWS), 6)
        self.profile_table.setHorizontalHeaderLabels(["Vel", "Vel max", "Turb", "Kp", "Ki", "Kd"])
        self.profile_table.setVerticalHeaderLabels([label for _segment_type, label in PROFILE_ROWS])
        self.profile_table.setMinimumHeight(150)
        self.profile_pid_delegate = PidGainDelegate(self.profile_table)
        for column in (3, 4, 5):
            self.profile_table.setItemDelegateForColumn(column, self.profile_pid_delegate)
        for column, width in enumerate([52, 64, 52, 62, 62, 62]):
            self.profile_table.setColumnWidth(column, width)
        for row, (segment_type, _label) in enumerate(PROFILE_ROWS):
            for column, value in enumerate(PROFILE_DEFAULTS[segment_type]):
                text = f"{value:.3f}" if isinstance(value, float) else str(value)
                self.profile_table.setItem(row, column, QTableWidgetItem(text))
        profile_layout.addWidget(self.profile_table)
        panel_layout.addWidget(profile_group)

        edit_group = QGroupBox("Segmentos")
        edit_layout = QVBoxLayout(edit_group)
        edit_layout.setContentsMargins(10, 16, 10, 10)
        self.segment_table = QTableWidget(0, 9)
        self.segment_table.setHorizontalHeaderLabels(["Tipo", "Inicio", "Fim", "Vel", "Vel max", "Turb", "Kp", "Ki", "Kd"])
        self.segment_table.verticalHeader().setVisible(False)
        self.segment_table.setMinimumHeight(250)
        self.segment_pid_delegate = PidGainDelegate(self.segment_table)
        for column in (6, 7, 8):
            self.segment_table.setItemDelegateForColumn(column, self.segment_pid_delegate)
        for column, width in enumerate([118, 62, 62, 52, 64, 52, 64, 64, 64]):
            self.segment_table.setColumnWidth(column, width)
        segment_buttons = QWidget()
        segment_button_layout = QHBoxLayout(segment_buttons)
        segment_button_layout.setContentsMargins(0, 0, 0, 0)
        self.add_segment_button = QPushButton("Adicionar")
        self.remove_segment_button = QPushButton("Remover")
        self.auto_detect_button = QPushButton("Detectar retas/curvas")
        segment_button_layout.addWidget(self.auto_detect_button)
        segment_button_layout.addWidget(self.add_segment_button)
        segment_button_layout.addWidget(self.remove_segment_button)
        edit_layout.addWidget(self.segment_table)
        edit_layout.addWidget(segment_buttons)
        panel_layout.addWidget(edit_group)

        state_group = QGroupBox("Estado")
        state_layout = QFormLayout(state_group)
        state_layout.setContentsMargins(16, 20, 16, 14)
        self.running_label = QLabel("-")
        self.target_label = QLabel("-")
        self.segment_state_label = QLabel("-")
        self.pid_label = QLabel("-")
        self.speed_label = QLabel("-")
        self.final_average_speed_label = QLabel("-")
        self.max_speed_label = QLabel("-")
        self.status_label = QLabel("-")
        state_layout.addRow("rodando", self.running_label)
        state_layout.addRow("target", self.target_label)
        state_layout.addRow("trecho", self.segment_state_label)
        state_layout.addRow("pid ativo", self.pid_label)
        state_layout.addRow("vel/turb", self.speed_label)
        state_layout.addRow("vel media final", self.final_average_speed_label)
        state_layout.addRow("vel maxima", self.max_speed_label)
        state_layout.addRow("status", self.status_label)
        panel_layout.addWidget(state_group)
        panel_layout.addStretch(1)

        side_scroll = QScrollArea()
        side_scroll.setWidgetResizable(True)
        side_scroll.setFrameShape(QScrollArea.NoFrame)
        side_scroll.setWidget(panel)
        side_scroll.setMinimumWidth(520)
        side_scroll.setMaximumWidth(620)
        root.addWidget(side_scroll, 0)

        map_group = QGroupBox("Mapa da pista")
        map_layout = QVBoxLayout(map_group)
        map_layout.setContentsMargins(8, 14, 8, 8)
        self.map_load_progress = QProgressBar()
        self.map_load_progress.setRange(0, 100)
        self.map_load_progress.hide()
        self.map_plot = pg.PlotWidget()
        self.map_plot.setMinimumSize(360, 320)
        self.map_plot.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.map_plot.setAspectLocked(True)
        self.map_plot.setMouseEnabled(x=False, y=False)
        self.map_plot.setMenuEnabled(False)
        self.map_plot.hideButtons()
        self.map_plot.showGrid(x=True, y=True, alpha=0.25)
        self.base_path = self.map_plot.plot(pen=pg.mkPen("#53616f", width=1, style=Qt.PenStyle.DotLine))
        self.target_item = pg.ScatterPlotItem(size=9, brush="#56ccf2", pen=pg.mkPen("#111820", width=1))
        self.map_plot.addItem(self.target_item)
        self.robot_marker = GpsRobotMarker()
        self.robot_marker.add_to(self.map_plot)
        map_layout.addWidget(self.map_load_progress)
        map_layout.addWidget(self.map_plot)
        root.addWidget(map_group, 1)

        self.refresh_button.clicked.connect(self.refresh_maps_requested)
        self.map_combo.currentIndexChanged.connect(self._emit_map_load)
        self.apply_button.clicked.connect(self._emit_apply)
        self.start_button.clicked.connect(self._emit_start)
        self.stop_button.clicked.connect(self.stop_requested)
        self.save_plan_file_button.clicked.connect(self._save_plan_file)
        self.load_plan_file_button.clicked.connect(self._load_plan_file)
        self.add_segment_button.clicked.connect(self._add_segment)
        self.remove_segment_button.clicked.connect(self._remove_selected_segment)
        self.auto_detect_button.clicked.connect(self._auto_detect_segments)
        self.auto_detect_check.toggled.connect(self._on_auto_detect_setting_changed)
        self.curve_threshold_input.valueChanged.connect(self._on_auto_detect_setting_changed)
        self.transition_extension_input.valueChanged.connect(self._on_auto_detect_setting_changed)
        self.profile_table.itemChanged.connect(self._on_profile_changed)
        self.segment_table.itemChanged.connect(self._on_table_changed)

        self._load_global_settings()

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
        self._clear_segment_curves()
        self.base_path.clear()
        self.target_item.clear()
        self.map_load_progress.setRange(0, max(1, total_points))
        self.map_load_progress.setValue(0)
        self.map_load_progress.setFormat("Carregando mapa: 0%")
        self.map_load_progress.show()
        self._load_segments_for_slot(slot, total_points)

    def set_loaded_chunk(self, offset: int, points: list[tuple[float, float]]) -> None:
        for index, point in enumerate(points):
            target = offset + index
            if 0 <= target < len(self._loaded_points):
                self._loaded_points[target] = point
        self._loading_received = min(len(self._loaded_points), max(self._loading_received, offset + len(points)))
        if self._loading_map:
            self.map_load_progress.setValue(self._loading_received)
            total = max(1, len(self._loaded_points))
            self.map_load_progress.setFormat(f"Carregando mapa: {int((self._loading_received * 100) / total)}%")
            return
        self._draw_map()

    def finish_loaded_map(self, slot: int) -> None:
        if self._loaded_slot != slot:
            return
        self._loading_map = False
        self.map_load_progress.hide()
        if self.auto_detect_check.isChecked():
            self._auto_detect_segments()
            return
        self._draw_map()

    def refresh(self, state: RobotState) -> None:
        self.running_label.setText("sim" if state.control_running else "nao")
        self.target_label.setText(f"{state.control_target_index}/{state.control_point_count}")
        self._update_segment_state_label(state)
        self.pid_label.setText(f"{state.control_kp:.3f}, {state.control_ki:.3f}, {state.control_kd:.3f}")
        self.speed_label.setText(
            f"{state.control_active_speed_percent} %/{state.control_speed_percent} % | turb {state.control_active_aux_percent} %"
        )
        if state.control_race_plan_average_speed_mps > 0.0:
            self.final_average_speed_label.setText(f"{state.control_race_plan_average_speed_mps:.3f} m/s")
        else:
            self.final_average_speed_label.setText("-")
        self.max_speed_label.setText(f"{state.control_max_speed_mps:.3f} m/s")
        if state.control_running:
            self._robot_pose_override = None
        self._draw_robot(state)
        if self._loaded_points and state.control_running:
            self.target_item.setData([state.control_target_x_m], [state.control_target_y_m])

    def _emit_map_load(self) -> None:
        if self._updating_maps:
            return
        data = self.map_combo.currentData()
        if data:
            slot = int(data["slot"])
            if slot != self._loaded_slot:
                self.map_load_requested.emit(slot)

    def _emit_apply(self) -> None:
        config = self._read_config()
        try:
            segments = self._validated_current_segments()
        except (TypeError, ValueError) as exc:
            self.status_label.setText(f"plano invalido: {exc}")
            return
        self._save_current_settings()
        self.apply_requested.emit(config, segments, self.battery_compensation_check.isChecked())
        self.status_label.setText("enviado")

    def _emit_start(self) -> None:
        data = self.map_combo.currentData()
        if not data:
            self.status_label.setText("sem mapa")
            return
        try:
            segments = self._validated_current_segments()
        except (TypeError, ValueError) as exc:
            self.status_label.setText(f"plano invalido: {exc}")
            return
        self.set_robot_at_first_point()
        self._save_current_settings()
        self.start_requested.emit(
            int(data["slot"]),
            int(self.start_speed_input.value()),
            self._read_config(),
            segments,
            self.battery_compensation_check.isChecked(),
        )
        self.status_label.setText("iniciando")

    @staticmethod
    def _map_signature(points: list[tuple[float, float]]) -> str:
        digest = hashlib.sha256()
        for x_m, y_m in points:
            x_value = float(x_m)
            y_value = float(y_m)
            if not math.isfinite(x_value) or not math.isfinite(y_value):
                raise ValueError("mapa possui ponto invalido")
            digest.update(struct.pack("<ff", x_value, y_value))
        return digest.hexdigest()

    def _loaded_map_context(self) -> tuple[dict, int]:
        data = self.map_combo.currentData()
        if (
            self._loading_map
            or self._loaded_slot is None
            or len(self._loaded_points) < 2
            or not isinstance(data, dict)
            or int(data.get("slot", -1)) != self._loaded_slot
        ):
            raise ValueError("carregue completamente o mapa antes de salvar ou carregar o plano")
        return data, len(self._loaded_points)

    def _read_profiles_for_file(self) -> dict[int, tuple[int, int, int, float, float, float]]:
        profiles: dict[int, tuple[int, int, int, float, float, float]] = {}
        for row, (segment_type, label) in enumerate(PROFILE_ROWS):
            try:
                raw_values = [self.profile_table.item(row, column).text() for column in range(6)]
            except AttributeError as exc:
                raise ValueError(f"perfil {label} incompleto") from exc
            profiles[segment_type] = (
                _require_json_int(_parse_decimal(raw_values[0]), f"perfil {label}.velocidade", 0, 100),
                _require_json_int(_parse_decimal(raw_values[1]), f"perfil {label}.velocidade maxima", 0, 100),
                _require_json_int(_parse_decimal(raw_values[2]), f"perfil {label}.turbina", 0, 100),
                _require_json_float(_parse_decimal(raw_values[3]), f"perfil {label}.kp", PID_MIN_GAIN, PID_MAX_GAIN),
                _require_json_float(_parse_decimal(raw_values[4]), f"perfil {label}.ki", PID_MIN_GAIN, PID_MAX_GAIN),
                _require_json_float(_parse_decimal(raw_values[5]), f"perfil {label}.kd", PID_MIN_GAIN, PID_MAX_GAIN),
            )
        return profiles

    def _build_plan_file_payload(self) -> dict:
        map_data, point_count = self._loaded_map_context()
        profiles = self._read_profiles_for_file()
        segments = self._validated_current_segments()

        segment_payload = [
            {
                "start_index": start,
                "end_index": end,
                "type": segment_type,
                "type_name": RACE_SEGMENT_LABELS.get(segment_type, str(segment_type)),
                "speed_percent": speed,
                "max_speed_percent": max_speed,
                "aux_percent": aux,
                "kp": kp,
                "ki": ki,
                "kd": kd,
            }
            for start, end, segment_type, speed, max_speed, aux, kp, ki, kd in segments
        ]

        profile_payload = [
            {
                "type": segment_type,
                "type_name": RACE_SEGMENT_LABELS.get(segment_type, str(segment_type)),
                "speed_percent": values[0],
                "max_speed_percent": values[1],
                "aux_percent": values[2],
                "kp": values[3],
                "ki": values[4],
                "kd": values[5],
            }
            for segment_type, values in profiles.items()
        ]

        return {
            "format": RACE_PLAN_FILE_FORMAT,
            "version": RACE_PLAN_FILE_VERSION,
            "map": {
                "slot": self._loaded_slot,
                "name": str(map_data.get("name", "")),
                "point_count": point_count,
                "distance_m": _require_json_float(
                    map_data.get("distance_m", 0.0), "map.distance_m", 0.0, 1000000.0
                ),
                "points_sha256": self._map_signature(self._loaded_points),
            },
            "config": {
                "start_speed_percent": int(self.start_speed_input.value()),
                "line_loss_odometry_enabled": self.line_loss_odometry_check.isChecked(),
                "battery_compensation_enabled": self.battery_compensation_check.isChecked(),
                "auto_detect_enabled": self.auto_detect_check.isChecked(),
                "curve_threshold_deg": int(self.curve_threshold_input.value()),
                "transition_extension_points": int(self.transition_extension_input.value()),
            },
            "profiles": profile_payload,
            "segments": segment_payload,
        }

    @staticmethod
    def _parse_profile_entries(raw_profiles: object) -> dict[int, tuple[int, int, int, float, float, float]]:
        if not isinstance(raw_profiles, list) or len(raw_profiles) != len(PROFILE_ROWS):
            raise ValueError(f"profiles deve conter {len(PROFILE_ROWS)} perfis")

        profiles: dict[int, tuple[int, int, int, float, float, float]] = {}
        valid_types = {segment_type for segment_type, _label in PROFILE_ROWS}
        for index, entry in enumerate(raw_profiles):
            if not isinstance(entry, dict):
                raise ValueError(f"profiles[{index}] deve ser objeto")
            segment_type = _require_json_int(entry.get("type"), f"profiles[{index}].type", 0, 255)
            if segment_type not in valid_types or segment_type in profiles:
                raise ValueError(f"profiles[{index}].type invalido ou duplicado")
            prefix = f"profiles[{index}]"
            profiles[segment_type] = (
                _require_json_int(entry.get("speed_percent"), f"{prefix}.speed_percent", 0, 100),
                _require_json_int(entry.get("max_speed_percent"), f"{prefix}.max_speed_percent", 0, 100),
                _require_json_int(entry.get("aux_percent"), f"{prefix}.aux_percent", 0, 100),
                _require_json_float(entry.get("kp"), f"{prefix}.kp", PID_MIN_GAIN, PID_MAX_GAIN),
                _require_json_float(entry.get("ki"), f"{prefix}.ki", PID_MIN_GAIN, PID_MAX_GAIN),
                _require_json_float(entry.get("kd"), f"{prefix}.kd", PID_MIN_GAIN, PID_MAX_GAIN),
            )

        if set(profiles) != valid_types:
            raise ValueError("arquivo nao contem todos os tipos de perfil")
        return profiles

    @staticmethod
    def _parse_segment_entries(raw_segments: object, point_count: int) -> list[tuple[int, int, int, int, int, int, float, float, float]]:
        if not isinstance(raw_segments, list) or not raw_segments:
            raise ValueError("segments deve conter ao menos um segmento")
        if len(raw_segments) > RACE_PLAN_MAX_SEGMENTS:
            raise ValueError(f"plano limitado a {RACE_PLAN_MAX_SEGMENTS} segmentos")

        segments: list[tuple[int, int, int, int, int, int, float, float, float]] = []
        expected_start = 1
        last_map_index = point_count - 1
        valid_types = set(RACE_SEGMENT_LABELS)
        for index, entry in enumerate(raw_segments):
            if not isinstance(entry, dict):
                raise ValueError(f"segments[{index}] deve ser objeto")
            prefix = f"segments[{index}]"
            start = _require_json_int(entry.get("start_index"), f"{prefix}.start_index", 1, last_map_index)
            end = _require_json_int(entry.get("end_index"), f"{prefix}.end_index", 1, last_map_index)
            segment_type = _require_json_int(entry.get("type"), f"{prefix}.type", 0, 255)
            if start > end:
                raise ValueError(f"{prefix} possui inicio maior que fim")
            if start != expected_start:
                raise ValueError(
                    f"{prefix}.start_index deve ser {expected_start} para manter cobertura continua"
                )
            if segment_type not in valid_types:
                raise ValueError(f"{prefix}.type invalido")
            segments.append(
                (
                    start,
                    end,
                    segment_type,
                    _require_json_int(entry.get("speed_percent"), f"{prefix}.speed_percent", 0, 100),
                    _require_json_int(entry.get("max_speed_percent"), f"{prefix}.max_speed_percent", 0, 100),
                    _require_json_int(entry.get("aux_percent"), f"{prefix}.aux_percent", 0, 100),
                    _require_json_float(entry.get("kp"), f"{prefix}.kp", PID_MIN_GAIN, PID_MAX_GAIN),
                    _require_json_float(entry.get("ki"), f"{prefix}.ki", PID_MIN_GAIN, PID_MAX_GAIN),
                    _require_json_float(entry.get("kd"), f"{prefix}.kd", PID_MIN_GAIN, PID_MAX_GAIN),
                )
            )
            expected_start = end + 1
        if expected_start != point_count:
            raise ValueError(
                f"ultimo segmento deve terminar no indice {last_map_index} para cobrir todo o mapa"
            )
        return segments

    def _parse_plan_file_payload(
        self,
        payload: object,
    ) -> tuple[
        dict,
        dict[int, tuple[int, int, int, float, float, float]],
        list[tuple[int, int, int, int, int, int, float, float, float]],
    ]:
        _map_data, point_count = self._loaded_map_context()
        if not isinstance(payload, dict):
            raise ValueError("raiz do arquivo deve ser um objeto")
        if payload.get("format") != RACE_PLAN_FILE_FORMAT:
            raise ValueError("formato de arquivo de plano desconhecido")
        version = _require_json_int(payload.get("version"), "version", 1, RACE_PLAN_FILE_VERSION)
        if version != RACE_PLAN_FILE_VERSION:
            raise ValueError(f"versao {version} nao suportada")

        map_info = payload.get("map")
        if not isinstance(map_info, dict):
            raise ValueError("map deve ser objeto")
        saved_point_count = _require_json_int(map_info.get("point_count"), "map.point_count", 2, 65535)
        if saved_point_count != point_count:
            raise ValueError(
                f"plano usa mapa com {saved_point_count} pontos; mapa atual possui {point_count}"
            )
        saved_signature = map_info.get("points_sha256")
        if not isinstance(saved_signature, str) or len(saved_signature) != 64:
            raise ValueError("map.points_sha256 invalido")
        if saved_signature.lower() != self._map_signature(self._loaded_points):
            saved_name = str(map_info.get("name", "outro mapa"))
            raise ValueError(f"plano pertence a uma geometria diferente ({saved_name})")

        raw_config = payload.get("config")
        if not isinstance(raw_config, dict):
            raise ValueError("config deve ser objeto")
        config = {
            "start_speed_percent": _require_json_int(
                raw_config.get("start_speed_percent"), "config.start_speed_percent", 1, 100
            ),
            "line_loss_odometry_enabled": _require_json_bool(
                raw_config.get("line_loss_odometry_enabled"), "config.line_loss_odometry_enabled"
            ),
            "battery_compensation_enabled": _require_json_bool(
                raw_config.get("battery_compensation_enabled"), "config.battery_compensation_enabled"
            ),
            "auto_detect_enabled": _require_json_bool(
                raw_config.get("auto_detect_enabled"), "config.auto_detect_enabled"
            ),
            "curve_threshold_deg": _require_json_int(
                raw_config.get("curve_threshold_deg"), "config.curve_threshold_deg", 2, 90
            ),
            "transition_extension_points": _require_json_int(
                raw_config.get("transition_extension_points"),
                "config.transition_extension_points",
                0,
                40,
            ),
        }
        profiles = self._parse_profile_entries(payload.get("profiles"))
        segments = self._parse_segment_entries(payload.get("segments"), point_count)
        return config, profiles, segments

    def _apply_plan_file_payload(
        self,
        config: dict,
        profiles: dict[int, tuple[int, int, int, float, float, float]],
        segments: list[tuple[int, int, int, int, int, int, float, float, float]],
    ) -> None:
        self._updating_table = True
        try:
            self.start_speed_input.setValue(config["start_speed_percent"])
            self.line_loss_odometry_check.setChecked(config["line_loss_odometry_enabled"])
            self.battery_compensation_check.setChecked(config["battery_compensation_enabled"])
            self.auto_detect_check.setChecked(config["auto_detect_enabled"])
            self.curve_threshold_input.setValue(config["curve_threshold_deg"])
            self.transition_extension_input.setValue(config["transition_extension_points"])

            for row, (segment_type, _label) in enumerate(PROFILE_ROWS):
                for column, value in enumerate(profiles[segment_type]):
                    text = f"{value:.{PID_DECIMALS}f}" if isinstance(value, float) else str(value)
                    self.profile_table.setItem(row, column, QTableWidgetItem(text))

            self.segment_table.setRowCount(0)
            for segment in segments:
                self._append_segment(segment)
        finally:
            self._updating_table = False

        self._save_current_settings()
        self._draw_map()

    def _plan_file_start_path(self, default_name: str = "") -> str:
        directory = self.settings.value("race_plan/file_directory", "", type=str)
        if not directory:
            return default_name
        return str(Path(directory) / default_name) if default_name else directory

    def _show_plan_file_error(self, title: str, message: str) -> None:
        self.status_label.setText(message)
        QMessageBox.warning(self, title, message)

    def _save_plan_file(self) -> None:
        try:
            payload = self._build_plan_file_payload()
        except (AttributeError, KeyError, TypeError, ValueError, OverflowError) as exc:
            self._show_plan_file_error("Salvar plano", str(exc))
            return

        map_name = str(payload["map"].get("name", "plano_corrida"))
        safe_name = "".join(character if character.isalnum() or character in "-_" else "_" for character in map_name)
        safe_name = safe_name.strip("_") or "plano_corrida"
        default_path = self._plan_file_start_path(f"{safe_name}_plano.json")
        file_name, _selected_filter = QFileDialog.getSaveFileName(
            self,
            "Salvar plano de corrida",
            default_path,
            "Plano de corrida JSON (*.json);;Todos os arquivos (*)",
        )
        if not file_name:
            return

        path = Path(file_name)
        if not path.suffix:
            path = path.with_suffix(".json")
        try:
            serialized = json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
            path.write_text(serialized, encoding="utf-8")
        except (OSError, UnicodeError, ValueError) as exc:
            self._show_plan_file_error("Salvar plano", f"nao foi possivel salvar: {exc}")
            return

        self.settings.setValue("race_plan/file_directory", str(path.parent))
        self._save_current_settings()
        self.status_label.setText(f"plano salvo: {path.name}")

    def _load_plan_file(self) -> None:
        try:
            self._loaded_map_context()
        except (TypeError, ValueError) as exc:
            self._show_plan_file_error("Carregar plano", str(exc))
            return

        file_name, _selected_filter = QFileDialog.getOpenFileName(
            self,
            "Carregar plano de corrida",
            self._plan_file_start_path(),
            "Plano de corrida JSON (*.json);;Todos os arquivos (*)",
        )
        if not file_name:
            return

        path = Path(file_name)
        try:
            if path.stat().st_size > 2 * 1024 * 1024:
                raise ValueError("arquivo de plano excede 2 MiB")
            payload = json.loads(path.read_text(encoding="utf-8"))
            config, profiles, segments = self._parse_plan_file_payload(payload)
        except (OSError, UnicodeError, json.JSONDecodeError, KeyError, TypeError, ValueError, OverflowError) as exc:
            self._show_plan_file_error("Carregar plano", f"arquivo invalido: {exc}")
            return

        self._apply_plan_file_payload(config, profiles, segments)
        self.settings.setValue("race_plan/file_directory", str(path.parent))
        self.status_label.setText(f"plano carregado: {path.name}")

    def _read_config(self) -> dict:
        return {
            "line_loss_odometry_enabled": self.line_loss_odometry_check.isChecked(),
        }

    def _read_segments(self) -> list[tuple[int, int, int, int, int, int, float, float, float]]:
        segments: list[tuple[int, int, int, int, int, int, float, float, float]] = []
        try:
            for row in range(self.segment_table.rowCount()):
                type_widget = self.segment_table.cellWidget(row, 0)
                segment_type = int(type_widget.currentData()) if isinstance(type_widget, QComboBox) else RACE_SEGMENT_NORMAL
                start = int(_parse_decimal(self.segment_table.item(row, 1).text()))
                end = int(_parse_decimal(self.segment_table.item(row, 2).text()))
                speed = int(_parse_decimal(self.segment_table.item(row, 3).text()))
                max_speed = int(_parse_decimal(self.segment_table.item(row, 4).text()))
                aux = int(_parse_decimal(self.segment_table.item(row, 5).text()))
                kp = _parse_decimal(self.segment_table.item(row, 6).text())
                ki = _parse_decimal(self.segment_table.item(row, 7).text())
                kd = _parse_decimal(self.segment_table.item(row, 8).text())
                segments.append((start, end, segment_type, speed, max_speed, aux, kp, ki, kd))
        except (AttributeError, TypeError, ValueError):
            return []
        segments.sort(key=lambda item: item[0])
        return segments

    def _validated_current_segments(self) -> list[tuple[int, int, int, int, int, int, float, float, float]]:
        _map_data, point_count = self._loaded_map_context()
        segments = self._read_segments()
        if not segments:
            raise ValueError("plano sem segmentos ou com valor invalido")
        payload = [
            {
                "start_index": start,
                "end_index": end,
                "type": segment_type,
                "speed_percent": speed,
                "max_speed_percent": max_speed,
                "aux_percent": aux,
                "kp": kp,
                "ki": ki,
                "kd": kd,
            }
            for start, end, segment_type, speed, max_speed, aux, kp, ki, kd in segments
        ]
        return self._parse_segment_entries(payload, point_count)

    def _segment_for_target(self, target_index: int) -> tuple[int, int, int, int, int, int, float, float, float] | None:
        for segment in self._read_segments():
            start, end, *_ = segment
            if int(start) <= target_index <= int(end):
                return segment
        return None

    def _update_segment_state_label(self, state: RobotState) -> None:
        if not self._loaded_points:
            self.segment_state_label.setText("-")
            self.segment_state_label.setStyleSheet("")
            return

        if state.control_race_segment_active:
            start = state.control_race_segment_start_index
            end = state.control_race_segment_end_index
            segment_type = state.control_race_segment_type
            speed = state.control_race_segment_speed_percent
            max_speed = state.control_race_segment_max_speed_percent
            aux = state.control_race_segment_aux_percent
        else:
            segment = self._segment_for_target(int(state.control_target_index))
            if segment is None:
                suffix = " | controle parado" if not state.control_running else ""
                self.segment_state_label.setText(f"Fora do plano{suffix}")
                self.segment_state_label.setStyleSheet("color: #9aa4ad;")
                return
            start, end, segment_type, speed, max_speed, aux, *_ = segment

        if not state.control_race_segment_active and not state.control_running:
            suffix = " | controle parado" if not state.control_running else ""
        else:
            suffix = ""
        label = RACE_SEGMENT_LABELS.get(int(segment_type), str(segment_type))
        color = SEGMENT_COLORS.get(int(segment_type), "#9aa4ad")
        self.segment_state_label.setText(f"{label} {start}-{end} | {speed}% max {max_speed}% | turb {aux}%{suffix}")
        self.segment_state_label.setStyleSheet(f"color: {color}; font-weight: 700;")

    def _add_segment(self) -> None:
        point_count = max(2, len(self._loaded_points))
        start = 1
        if self.segment_table.rowCount() > 0:
            try:
                start = min(point_count - 1, int(_parse_decimal(self.segment_table.item(self.segment_table.rowCount() - 1, 2).text())) + 1)
            except (AttributeError, TypeError, ValueError):
                start = 1
        end = min(point_count - 1, max(start, start + 10))
        speed, max_speed, aux, kp, ki, kd = self._profile_defaults()[RACE_SEGMENT_NORMAL]
        self._updating_table = True
        self._append_segment((start, end, RACE_SEGMENT_NORMAL, speed, max_speed, aux, kp, ki, kd))
        self._updating_table = False
        self._save_current_settings()
        self._draw_map()

    def _remove_selected_segment(self) -> None:
        row = self.segment_table.currentRow()
        if row >= 0:
            self.segment_table.removeRow(row)
            self._save_current_settings()
            self._draw_map()

    def _auto_detect_segments(self, *_args) -> None:
        if len(self._loaded_points) < 3:
            self.status_label.setText("mapa curto")
            return

        point_count = len(self._loaded_points)
        target_count = point_count - 1
        profile_defaults = self._profile_defaults()
        segment_types = self._detect_segment_types(np.array(self._loaded_points, dtype=float))

        stop_segments = []
        for segment in self._read_segments():
            start, end, segment_type, speed, max_speed, aux, kp, ki, kd = segment
            if segment_type != RACE_SEGMENT_STOP:
                continue
            start_index = max(1, min(target_count, int(start)))
            end_index = max(start_index, min(target_count, int(end)))
            stop_segments.append((start_index, end_index, speed, max_speed, aux, kp, ki, kd))
            for index in range(start_index, end_index + 1):
                segment_types[index] = RACE_SEGMENT_STOP

        self._apply_intersection_extension(segment_types, target_count)
        runs = self._runs_from_types(segment_types, target_count)
        runs = self._limit_run_count(runs)
        segments = [
            self._segment_with_profile(run_start, run_end, run_type, profile_defaults, stop_segments)
            for run_start, run_end, run_type in runs
        ]

        self._replace_segments(segments)
        curve_count = sum(1 for _, _, segment_type, *_ in segments if segment_type == RACE_SEGMENT_CURVE)
        intersection_count = sum(1 for _, _, segment_type, *_ in segments if segment_type == RACE_SEGMENT_INTERSECTION)
        self.status_label.setText(f"auto: {len(segments)} seg, {curve_count} curva, {intersection_count} inter")

    def _append_segment(self, segment: tuple[int, int, int, int, int, int, float, float, float]) -> None:
        row = self.segment_table.rowCount()
        self.segment_table.insertRow(row)
        type_combo = QComboBox()
        for value, label in RACE_SEGMENT_LABELS.items():
            type_combo.addItem(label, value)
        type_combo.setCurrentIndex(max(0, type_combo.findData(int(segment[2]))))
        type_combo.currentIndexChanged.connect(self._on_type_changed)
        self.segment_table.setCellWidget(row, 0, type_combo)
        for column, value in enumerate([segment[0], segment[1], segment[3], segment[4], segment[5], segment[6], segment[7], segment[8]], start=1):
            text = f"{value:.3f}" if isinstance(value, float) else str(value)
            self.segment_table.setItem(row, column, QTableWidgetItem(text))

    def _replace_segments(self, segments: list[tuple[int, int, int, int, int, int, float, float, float]]) -> None:
        self._updating_table = True
        self.segment_table.setRowCount(0)
        for segment in segments:
            self._append_segment(segment)
        self._updating_table = False
        self._save_current_settings()
        self._draw_map()

    def _profile_defaults(self) -> dict[int, tuple[int, int, int, float, float, float]]:
        defaults = dict(PROFILE_DEFAULTS)
        try:
            for row, (segment_type, _label) in enumerate(PROFILE_ROWS):
                speed = int(_parse_decimal(self.profile_table.item(row, 0).text()))
                max_speed = int(_parse_decimal(self.profile_table.item(row, 1).text()))
                aux = int(_parse_decimal(self.profile_table.item(row, 2).text()))
                kp = _parse_decimal(self.profile_table.item(row, 3).text())
                ki = _parse_decimal(self.profile_table.item(row, 4).text())
                kd = _parse_decimal(self.profile_table.item(row, 5).text())
                defaults[segment_type] = (
                    max(0, min(100, speed)),
                    max(0, min(100, max_speed)),
                    max(0, min(100, aux)),
                    kp,
                    ki,
                    kd,
                )
        except (AttributeError, TypeError, ValueError):
            pass
        return defaults

    def _normalize_loaded_segment(
        self,
        segment: tuple[int, int, int, int, int, int, float, float, float],
    ) -> tuple[int, int, int, int, int, int, float, float, float]:
        start, end, segment_type, speed, max_speed, aux, kp, ki, kd = segment
        normal_speed = int(self.start_speed_input.value())
        legacy_curve_speed = max(1, int(round(normal_speed * 0.65)))
        default_pid = all(
            math.isclose(float(value), float(default), abs_tol=0.001)
            for value, default in zip((kp, ki, kd), DEFAULT_PID)
        )
        if (
            int(segment_type) == RACE_SEGMENT_CURVE
            and int(speed) == legacy_curve_speed
            and int(max_speed) == 100
            and int(aux) == 0
            and default_pid
        ):
            speed = normal_speed
        return (start, end, segment_type, speed, max_speed, aux, kp, ki, kd)

    def _segment_with_profile(
        self,
        start: int,
        end: int,
        segment_type: int,
        defaults: dict[int, tuple[int, int, int, float, float, float]],
        stop_segments: list[tuple[int, int, int, int, int, float, float, float]],
    ) -> tuple[int, int, int, int, int, int, float, float, float]:
        if segment_type == RACE_SEGMENT_STOP:
            for stop_start, stop_end, speed, max_speed, aux, kp, ki, kd in stop_segments:
                if start >= stop_start and end <= stop_end:
                    return (start, end, segment_type, speed, max_speed, aux, kp, ki, kd)
        speed, max_speed, aux, kp, ki, kd = defaults.get(segment_type, defaults[RACE_SEGMENT_NORMAL])
        return (start, end, segment_type, speed, max_speed, aux, kp, ki, kd)

    def _detect_segment_types(self, points: np.ndarray) -> list[int]:
        point_count = len(points)
        target_count = point_count - 1
        threshold_rad = math.radians(float(self.curve_threshold_input.value()))
        window = max(1, min(AUTO_DETECT_WINDOW_POINTS, target_count // 3))
        padding = max(1, min(AUTO_DETECT_PADDING_POINTS, target_count // 4))
        curve_flags = [False] * point_count

        for index in range(1, point_count - 1):
            previous_index = max(0, index - window)
            next_index = min(point_count - 1, index + window)
            if previous_index == index or next_index == index:
                continue

            before = points[index] - points[previous_index]
            after = points[next_index] - points[index]
            before_norm = float(np.linalg.norm(before))
            after_norm = float(np.linalg.norm(after))
            if before_norm < 0.005 or after_norm < 0.005:
                continue

            before_heading = math.atan2(float(before[1]), float(before[0]))
            after_heading = math.atan2(float(after[1]), float(after[0]))
            turn = abs(self._wrap_pi(after_heading - before_heading))
            if turn >= threshold_rad:
                start = max(1, index - padding)
                end = min(target_count, index + padding)
                for target_index in range(start, end + 1):
                    curve_flags[target_index] = True

        self._smooth_curve_flags(curve_flags, target_count)
        return [
            RACE_SEGMENT_CURVE if index <= target_count and curve_flags[index] else RACE_SEGMENT_NORMAL
            for index in range(point_count)
        ]

    def _apply_intersection_extension(self, segment_types: list[int], target_count: int) -> None:
        extension = int(self.transition_extension_input.value())
        if extension <= 0:
            return

        base_types = list(segment_types)
        for index in range(2, target_count + 1):
            left = base_types[index - 1]
            right = base_types[index]
            if {left, right} != {RACE_SEGMENT_NORMAL, RACE_SEGMENT_CURVE}:
                continue
            start = max(1, index - extension)
            end = min(target_count, index + extension - 1)
            for target_index in range(start, end + 1):
                if segment_types[target_index] in (RACE_SEGMENT_NORMAL, RACE_SEGMENT_CURVE):
                    segment_types[target_index] = RACE_SEGMENT_INTERSECTION

    def _smooth_curve_flags(self, flags: list[bool], target_count: int) -> None:
        min_run = max(AUTO_DETECT_MIN_RUN_POINTS, AUTO_DETECT_PADDING_POINTS * 2)
        index = 1
        while index <= target_count:
            value = flags[index]
            start = index
            while index + 1 <= target_count and flags[index + 1] == value:
                index += 1
            end = index
            length = end - start + 1
            previous_is_curve = start > 1 and flags[start - 1]
            next_is_curve = end < target_count and flags[end + 1]
            if value and length < AUTO_DETECT_MIN_RUN_POINTS:
                for fill_index in range(start, end + 1):
                    flags[fill_index] = False
            elif not value and previous_is_curve and next_is_curve and length <= min_run:
                for fill_index in range(start, end + 1):
                    flags[fill_index] = True
            index += 1

    def _runs_from_types(self, segment_types: list[int], target_count: int) -> list[tuple[int, int, int]]:
        runs: list[tuple[int, int, int]] = []
        start = 1
        current_type = segment_types[1]
        for index in range(2, target_count + 1):
            if segment_types[index] != current_type:
                runs.append((start, index - 1, current_type))
                start = index
                current_type = segment_types[index]
        runs.append((start, target_count, current_type))
        return self._merge_adjacent_runs(runs)

    def _limit_run_count(self, runs: list[tuple[int, int, int]]) -> list[tuple[int, int, int]]:
        limited = self._merge_adjacent_runs(runs)
        while len(limited) > RACE_PLAN_MAX_SEGMENTS:
            candidates = [
                (end - start + 1, index)
                for index, (start, end, segment_type) in enumerate(limited)
                if segment_type != RACE_SEGMENT_STOP
            ]
            if not candidates:
                break
            _, index = min(candidates)
            if index == 0:
                left_start, _, _ = limited[index]
                _, right_end, right_type = limited[index + 1]
                limited[index:index + 2] = [(left_start, right_end, right_type)]
            elif index == len(limited) - 1:
                left_start, _, left_type = limited[index - 1]
                _, right_end, _ = limited[index]
                limited[index - 1:index + 1] = [(left_start, right_end, left_type)]
            else:
                previous_length = limited[index - 1][1] - limited[index - 1][0]
                next_length = limited[index + 1][1] - limited[index + 1][0]
                if previous_length >= next_length:
                    left_start, _, left_type = limited[index - 1]
                    _, right_end, _ = limited[index]
                    limited[index - 1:index + 1] = [(left_start, right_end, left_type)]
                else:
                    left_start, _, _ = limited[index]
                    _, right_end, right_type = limited[index + 1]
                    limited[index:index + 2] = [(left_start, right_end, right_type)]
            limited = self._merge_adjacent_runs(limited)
        return limited

    @staticmethod
    def _merge_adjacent_runs(runs: list[tuple[int, int, int]]) -> list[tuple[int, int, int]]:
        if not runs:
            return []
        merged = [runs[0]]
        for start, end, segment_type in runs[1:]:
            previous_start, previous_end, previous_type = merged[-1]
            if segment_type == previous_type and start <= previous_end + 1:
                merged[-1] = (previous_start, end, previous_type)
            else:
                merged.append((start, end, segment_type))
        return merged

    @staticmethod
    def _wrap_pi(angle: float) -> float:
        while angle > math.pi:
            angle -= math.tau
        while angle < -math.pi:
            angle += math.tau
        return angle

    def _on_type_changed(self, *_args) -> None:
        if self._updating_table:
            return
        combo = self.sender()
        if isinstance(combo, QComboBox):
            row = -1
            for candidate in range(self.segment_table.rowCount()):
                if self.segment_table.cellWidget(candidate, 0) is combo:
                    row = candidate
                    break
            segment_type = int(combo.currentData())
            if row >= 0:
                self._updating_table = True
                if segment_type == RACE_SEGMENT_STOP:
                    self.segment_table.setItem(row, 3, QTableWidgetItem("0"))
                    self.segment_table.setItem(row, 4, QTableWidgetItem("0"))
                    self.segment_table.setItem(row, 5, QTableWidgetItem("100"))
                else:
                    defaults = self._profile_defaults()
                    speed_item = self.segment_table.item(row, 3)
                    try:
                        current_speed = int(_parse_decimal(speed_item.text())) if speed_item is not None else 0
                    except ValueError:
                        current_speed = 0
                    if current_speed <= 0:
                        default_speed = defaults.get(segment_type, defaults[RACE_SEGMENT_NORMAL])[0]
                        self.segment_table.setItem(row, 3, QTableWidgetItem(str(default_speed)))
                    max_item = self.segment_table.item(row, 4)
                    try:
                        current_max = int(_parse_decimal(max_item.text())) if max_item is not None else 0
                    except ValueError:
                        current_max = 0
                    if current_max <= 0:
                        self.segment_table.setItem(row, 4, QTableWidgetItem("100"))
                    aux_item = self.segment_table.item(row, 5)
                    try:
                        current_aux = int(_parse_decimal(aux_item.text())) if aux_item is not None else 100
                    except ValueError:
                        current_aux = 100
                    if current_aux >= 100:
                        self.segment_table.setItem(row, 5, QTableWidgetItem("0"))
                self._updating_table = False
        self._save_current_settings()
        self._draw_map()

    def _on_auto_detect_setting_changed(self, *_args) -> None:
        if self._updating_table:
            return
        self._save_current_settings()
        if self.auto_detect_check.isChecked() and self._loaded_points and not self._loading_map:
            self._auto_detect_segments()

    def _on_table_changed(self, *_args) -> None:
        if self._updating_table:
            return
        self._save_current_settings()
        self._draw_map()

    def _on_profile_changed(self, *_args) -> None:
        if self._updating_table:
            return
        self._save_current_settings()
        if self.auto_detect_check.isChecked() and self._loaded_points and not self._loading_map:
            self._auto_detect_segments()

    def _load_segments_for_slot(self, slot: int, total_points: int) -> None:
        self._updating_table = True
        self.segment_table.setRowCount(0)
        raw = self.settings.value(f"race_plan/{slot}/segments", "", type=str)
        loaded = []
        if raw:
            try:
                loaded = json.loads(raw)
            except json.JSONDecodeError:
                loaded = []
        if not loaded and total_points > 1:
            speed, max_speed, aux, kp, ki, kd = self._profile_defaults()[RACE_SEGMENT_NORMAL]
            loaded = [[1, total_points - 1, RACE_SEGMENT_NORMAL, speed, max_speed, aux, kp, ki, kd]]
        for item in loaded:
            if len(item) == 8:
                start, end, segment_type, speed, aux, kp, ki, kd = item
                max_speed = 0 if int(segment_type) == RACE_SEGMENT_STOP else 100
                segment = self._normalize_loaded_segment((start, end, segment_type, speed, max_speed, aux, kp, ki, kd))
                self._append_segment(segment)
            elif len(item) == 9:
                self._append_segment(self._normalize_loaded_segment(tuple(item)))
        self._updating_table = False
        self._draw_map()

    def _load_global_settings(self) -> None:
        self.start_speed_input.setValue(self.settings.value("race_plan/start_speed", 50, type=int))
        self.line_loss_odometry_check.setChecked(self.settings.value("race_plan/line_loss_odometry", True, type=bool))
        self.battery_compensation_check.setChecked(self.settings.value("race_plan/battery_compensation", False, type=bool))
        self.auto_detect_check.setChecked(self.settings.value("race_plan/auto_detect", False, type=bool))
        self.curve_threshold_input.setValue(self.settings.value("race_plan/curve_threshold_deg", 10, type=int))
        self.transition_extension_input.setValue(self.settings.value("race_plan/transition_extension_points", 4, type=int))
        raw_profiles = self.settings.value("race_plan/profile_defaults", "", type=str)
        if not raw_profiles:
            return
        try:
            profiles = json.loads(raw_profiles)
        except json.JSONDecodeError:
            return
        if not isinstance(profiles, dict):
            return
        self._updating_table = True
        for row, (segment_type, _label) in enumerate(PROFILE_ROWS):
            values = profiles.get(str(segment_type), PROFILE_DEFAULTS[segment_type])
            if len(values) != 6:
                continue
            for column, value in enumerate(values):
                text = f"{value:.3f}" if isinstance(value, float) else str(value)
                self.profile_table.setItem(row, column, QTableWidgetItem(text))
        self._updating_table = False

    def _save_current_settings(self) -> None:
        self.settings.setValue("race_plan/start_speed", int(self.start_speed_input.value()))
        self.settings.setValue("race_plan/line_loss_odometry", self.line_loss_odometry_check.isChecked())
        self.settings.setValue("race_plan/battery_compensation", self.battery_compensation_check.isChecked())
        self.settings.setValue("race_plan/auto_detect", self.auto_detect_check.isChecked())
        self.settings.setValue("race_plan/curve_threshold_deg", int(self.curve_threshold_input.value()))
        self.settings.setValue("race_plan/transition_extension_points", int(self.transition_extension_input.value()))
        profiles = {
            str(segment_type): list(values)
            for segment_type, values in self._profile_defaults().items()
        }
        self.settings.setValue("race_plan/profile_defaults", json.dumps(profiles))
        if self._loaded_slot is not None:
            self.settings.setValue(f"race_plan/{self._loaded_slot}/segments", json.dumps(self._read_segments()))

    def first_point_pose(self) -> tuple[float, float, float] | None:
        if not self._loaded_points or self._loading_map:
            return None
        x_m, y_m = self._loaded_points[0]
        heading_rad = 0.0
        if len(self._loaded_points) > 1:
            next_x, next_y = self._loaded_points[1]
            heading_rad = float(np.arctan2(next_y - y_m, next_x - x_m))
        return (float(x_m), float(y_m), heading_rad)

    def set_robot_at_first_point(self) -> tuple[float, float, float] | None:
        pose = self.first_point_pose()
        if pose is None:
            return None
        self._robot_pose_override = pose
        self.robot_marker.set_pose(*pose)
        return pose

    def _draw_robot(self, state: RobotState) -> None:
        if self._robot_pose_override is not None and not state.control_running:
            self.robot_marker.set_pose(*self._robot_pose_override)
            return
        if state.control_running and state.control_map_pose_valid:
            self.robot_marker.set_pose(
                state.control_map_x_m,
                state.control_map_y_m,
                state.control_map_heading_rad,
            )
            return
        self.robot_marker.set_pose(state.x_m, state.y_m, state.heading_rad)

    def _draw_map(self) -> None:
        self._clear_segment_curves()
        if not self._loaded_points or self._loading_map:
            self.base_path.clear()
            self.target_item.clear()
            return
        points = np.array(self._loaded_points)
        self.base_path.setData(points[:, 0], points[:, 1])
        for start, end, segment_type, *_ in self._read_segments():
            start_index = max(1, min(len(self._loaded_points) - 1, int(start)))
            end_index = max(start_index, min(len(self._loaded_points) - 1, int(end)))
            segment_points = np.array(self._loaded_points[start_index - 1:end_index + 1])
            curve = self.map_plot.plot(
                segment_points[:, 0],
                segment_points[:, 1],
                pen=pg.mkPen(SEGMENT_COLORS.get(segment_type, "#27ae60"), width=4),
            )
            self._segment_curves.append(curve)
        self.target_item.setData([points[-1, 0]], [points[-1, 1]])
        self._fit_map()

    def _clear_segment_curves(self) -> None:
        for curve in self._segment_curves:
            self.map_plot.removeItem(curve)
        self._segment_curves = []

    def _fit_map(self) -> None:
        if not self._loaded_points:
            return
        xs = [point[0] for point in self._loaded_points]
        ys = [point[1] for point in self._loaded_points]
        width = max(max(xs) - min(xs), 0.2)
        height = max(max(ys) - min(ys), 0.2)
        self.map_plot.setRange(
            xRange=(min(xs) - (width * 0.12), max(xs) + (width * 0.12)),
            yRange=(min(ys) - (height * 0.12), max(ys) + (height * 0.12)),
            padding=0.0,
        )
