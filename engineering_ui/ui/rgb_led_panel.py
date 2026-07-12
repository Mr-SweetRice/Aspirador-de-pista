from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QCheckBox, QComboBox, QFormLayout, QGroupBox, QLabel, QPushButton, QSlider, QVBoxLayout, QWidget

from ble.protocol import RGB_LED_MODE_BATTERY, RGB_LED_MODE_LABELS, RGB_LED_MODE_MANUAL
from telemetry.state import RobotState


class RgbLedPanel(QWidget):
    enabled_requested = Signal(bool)
    mode_requested = Signal(int)
    manual_requested = Signal(int, int, int, int)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        root = QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)

        group = QGroupBox("LED RGB")
        form = QFormLayout(group)
        self.enabled_check = QCheckBox("Habilitado")
        self.mode_combo = QComboBox()
        for mode, label in RGB_LED_MODE_LABELS.items():
            self.mode_combo.addItem(label, mode)
        self.red_slider = self._make_slider()
        self.green_slider = self._make_slider()
        self.blue_slider = self._make_slider()
        self.intensity_slider = self._make_slider()
        self.intensity_slider.setValue(32)
        self.apply_button = QPushButton("Aplicar cor manual")
        self.preview = QLabel("-")

        form.addRow("Estado", self.enabled_check)
        form.addRow("Funcao", self.mode_combo)
        form.addRow("Vermelho", self.red_slider)
        form.addRow("Verde", self.green_slider)
        form.addRow("Azul", self.blue_slider)
        form.addRow("Intensidade", self.intensity_slider)
        form.addRow(self.apply_button)
        form.addRow("Atual", self.preview)
        root.addWidget(group)
        root.addStretch(1)

        self.enabled_check.toggled.connect(self.enabled_requested)
        self.mode_combo.currentIndexChanged.connect(self._emit_mode)
        self.apply_button.clicked.connect(self._emit_manual)

    def refresh(self, state: RobotState) -> None:
        if self.enabled_check.isChecked() != state.rgb_led_enabled:
            self.enabled_check.blockSignals(True)
            self.enabled_check.setChecked(state.rgb_led_enabled)
            self.enabled_check.blockSignals(False)

        if self.mode_combo.currentData() != state.rgb_led_mode:
            self.mode_combo.blockSignals(True)
            for index in range(self.mode_combo.count()):
                if self.mode_combo.itemData(index) == state.rgb_led_mode:
                    self.mode_combo.setCurrentIndex(index)
                    break
            self.mode_combo.blockSignals(False)

        self.preview.setText(
            f"{RGB_LED_MODE_LABELS.get(state.rgb_led_mode, state.rgb_led_mode)} | "
            f"R {state.rgb_led_red} G {state.rgb_led_green} B {state.rgb_led_blue} I {state.rgb_led_intensity}"
        )

    def _emit_mode(self) -> None:
        data = self.mode_combo.currentData()
        if data is not None:
            self.mode_requested.emit(int(data))

    def _emit_manual(self) -> None:
        self.manual_requested.emit(
            self.red_slider.value(),
            self.green_slider.value(),
            self.blue_slider.value(),
            self.intensity_slider.value(),
        )

    @staticmethod
    def _make_slider() -> QSlider:
        slider = QSlider(Qt.Horizontal)
        slider.setRange(0, 255)
        slider.setValue(255)
        return slider
