from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QLabel,
    QVBoxLayout,
    QWidget,
)

from telemetry.state import RobotState


class SafetyPanel(QWidget):
    collision_enabled_requested = Signal(bool)
    battery_block_enabled_requested = Signal(bool)
    line_loss_enabled_requested = Signal(bool)
    ble_loss_enabled_requested = Signal(bool)
    roll_limit_requested = Signal(float)
    battery_block_percent_requested = Signal(float)
    line_loss_timeout_requested = Signal(float)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._updating = False

        root = QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        collision_group = QGroupBox("Colisao por roll")
        collision_layout = QFormLayout(collision_group)
        collision_layout.setContentsMargins(16, 20, 16, 14)
        self.collision_enabled_check = QCheckBox("Habilitar parada por inclinacao")
        self.roll_limit_input = QDoubleSpinBox()
        self.roll_limit_input.setRange(1.0, 90.0)
        self.roll_limit_input.setDecimals(1)
        self.roll_limit_input.setSingleStep(0.5)
        self.roll_limit_input.setValue(6.0)
        self.roll_limit_input.setSuffix(" deg")
        collision_layout.addRow(self.collision_enabled_check)
        collision_layout.addRow("Limite roll", self.roll_limit_input)
        root.addWidget(collision_group)

        battery_group = QGroupBox("Bloqueio por bateria")
        battery_layout = QFormLayout(battery_group)
        battery_layout.setContentsMargins(16, 20, 16, 14)
        self.battery_block_enabled_check = QCheckBox("Bloquear motores por bateria baixa")
        self.battery_block_input = QDoubleSpinBox()
        self.battery_block_input.setRange(0.0, 100.0)
        self.battery_block_input.setDecimals(1)
        self.battery_block_input.setSingleStep(1.0)
        self.battery_block_input.setValue(10.0)
        self.battery_block_input.setSuffix(" %")
        battery_layout.addRow(self.battery_block_enabled_check)
        battery_layout.addRow("Bloquear abaixo de", self.battery_block_input)
        root.addWidget(battery_group)

        line_group = QGroupBox("Perda de linha")
        line_layout = QFormLayout(line_group)
        line_layout.setContentsMargins(16, 20, 16, 14)
        self.line_loss_enabled_check = QCheckBox("Parar se perder a linha")
        self.line_loss_timeout_input = QDoubleSpinBox()
        self.line_loss_timeout_input.setRange(0.1, 10.0)
        self.line_loss_timeout_input.setDecimals(1)
        self.line_loss_timeout_input.setSingleStep(0.1)
        self.line_loss_timeout_input.setValue(1.0)
        self.line_loss_timeout_input.setSuffix(" s")
        line_layout.addRow(self.line_loss_enabled_check)
        line_layout.addRow("Tempo sem linha", self.line_loss_timeout_input)
        root.addWidget(line_group)

        ble_group = QGroupBox("Perda de BLE")
        ble_layout = QFormLayout(ble_group)
        ble_layout.setContentsMargins(16, 20, 16, 14)
        self.ble_loss_enabled_check = QCheckBox("Parar se desconectar do BLE")
        ble_layout.addRow(self.ble_loss_enabled_check)
        root.addWidget(ble_group)

        status_group = QGroupBox("Estado")
        status_layout = QFormLayout(status_group)
        status_layout.setContentsMargins(16, 20, 16, 14)
        self.blocked_label = QLabel("-")
        self.collision_label = QLabel("-")
        self.battery_label = QLabel("-")
        self.line_loss_label = QLabel("-")
        self.ble_loss_label = QLabel("-")
        self.ble_connected_label = QLabel("-")
        self.line_visible_label = QLabel("-")
        self.line_loss_elapsed_label = QLabel("-")
        self.roll_label = QLabel("-")
        self.battery_percent_label = QLabel("-")
        status_layout.addRow("motores bloqueados", self.blocked_label)
        status_layout.addRow("colisao ativa", self.collision_label)
        status_layout.addRow("bateria ativa", self.battery_label)
        status_layout.addRow("perda linha ativa", self.line_loss_label)
        status_layout.addRow("perda BLE ativa", self.ble_loss_label)
        status_layout.addRow("BLE conectado", self.ble_connected_label)
        status_layout.addRow("linha visivel", self.line_visible_label)
        status_layout.addRow("tempo sem linha", self.line_loss_elapsed_label)
        status_layout.addRow("roll atual", self.roll_label)
        status_layout.addRow("bateria atual", self.battery_percent_label)
        root.addWidget(status_group)
        root.addStretch(1)

        self.collision_enabled_check.toggled.connect(self._emit_collision_enabled)
        self.battery_block_enabled_check.toggled.connect(self._emit_battery_block_enabled)
        self.line_loss_enabled_check.toggled.connect(self._emit_line_loss_enabled)
        self.ble_loss_enabled_check.toggled.connect(self._emit_ble_loss_enabled)
        self.roll_limit_input.editingFinished.connect(self._emit_roll_limit)
        self.battery_block_input.editingFinished.connect(self._emit_battery_block_percent)
        self.line_loss_timeout_input.editingFinished.connect(self._emit_line_loss_timeout)

    def refresh(self, state: RobotState) -> None:
        self._updating = True
        self.collision_enabled_check.setChecked(state.safety_collision_enabled)
        self.battery_block_enabled_check.setChecked(state.safety_battery_block_enabled)
        self.line_loss_enabled_check.setChecked(state.safety_line_loss_enabled)
        self.ble_loss_enabled_check.setChecked(state.safety_ble_loss_enabled)
        if not self.roll_limit_input.hasFocus():
            self.roll_limit_input.setValue(state.safety_roll_limit_deg)
        if not self.battery_block_input.hasFocus():
            self.battery_block_input.setValue(state.safety_battery_block_percent)
        if not self.line_loss_timeout_input.hasFocus():
            self.line_loss_timeout_input.setValue(state.safety_line_loss_timeout_s)
        self._updating = False

        self.blocked_label.setText("sim" if state.safety_motors_blocked else "nao")
        self.collision_label.setText("sim" if state.safety_collision_active else "nao")
        self.battery_label.setText("sim" if state.safety_battery_block_active else "nao")
        self.line_loss_label.setText("sim" if state.safety_line_loss_active else "nao")
        self.ble_loss_label.setText("sim" if state.safety_ble_loss_active else "nao")
        self.ble_connected_label.setText("sim" if state.safety_ble_connected else "nao")
        self.line_visible_label.setText("sim" if state.safety_line_visible else "nao")
        self.line_loss_elapsed_label.setText(f"{state.safety_line_loss_elapsed_s:.2f} s")
        self.roll_label.setText(f"{state.safety_current_roll_deg:.1f} deg")
        self.battery_percent_label.setText(f"{state.safety_current_battery_percent:.1f} %")

    def _emit_collision_enabled(self, enabled: bool) -> None:
        if not self._updating:
            self.collision_enabled_requested.emit(enabled)

    def _emit_battery_block_enabled(self, enabled: bool) -> None:
        if not self._updating:
            self.battery_block_enabled_requested.emit(enabled)

    def _emit_line_loss_enabled(self, enabled: bool) -> None:
        if not self._updating:
            self.line_loss_enabled_requested.emit(enabled)

    def _emit_ble_loss_enabled(self, enabled: bool) -> None:
        if not self._updating:
            self.ble_loss_enabled_requested.emit(enabled)

    def _emit_roll_limit(self) -> None:
        if not self._updating:
            self.roll_limit_requested.emit(float(self.roll_limit_input.value()))

    def _emit_battery_block_percent(self) -> None:
        if not self._updating:
            self.battery_block_percent_requested.emit(float(self.battery_block_input.value()))

    def _emit_line_loss_timeout(self) -> None:
        if not self._updating:
            self.line_loss_timeout_requested.emit(float(self.line_loss_timeout_input.value()))
