from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QLabel,
    QPushButton,
    QSizePolicy,
    QSlider,
    QVBoxLayout,
    QWidget,
)


class ControlPanel(QWidget):
    stop_requested = Signal()
    left_pwm_changed = Signal(int)
    right_pwm_changed = Signal(int)
    aux_pwm_changed = Signal(int)
    reset_encoders_requested = Signal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setSpacing(10)
        layout.setContentsMargins(0, 0, 0, 0)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)

        stop = QPushButton("PARADA DE EMERGENCIA")
        stop.setObjectName("stopButton")
        stop.setToolTip("Corta os motores imediatamente")
        stop.setMinimumHeight(54)
        stop.clicked.connect(self.stop_requested)
        layout.addWidget(stop)

        motors = QGroupBox("Motores")
        motor_layout = QGridLayout(motors)
        self.left_slider, self.left_value = self._make_slider()
        self.right_slider, self.right_value = self._make_slider()

        motor_layout.addWidget(QLabel("Esquerdo"), 0, 0)
        motor_layout.addWidget(self.left_slider, 0, 1)
        motor_layout.addWidget(self.left_value, 0, 2)
        motor_layout.addWidget(QLabel("Direito"), 1, 0)
        motor_layout.addWidget(self.right_slider, 1, 1)
        motor_layout.addWidget(self.right_value, 1, 2)
        layout.addWidget(motors)

        aux = QGroupBox("Motor auxiliar")
        aux_layout = QGridLayout(aux)
        self.aux_slider = QSlider(Qt.Horizontal)
        self.aux_slider.setRange(0, 100)
        self.aux_value = QLabel("0%")
        self.aux_slider.valueChanged.connect(lambda value: self.aux_value.setText(f"{value}%"))
        self.aux_slider.valueChanged.connect(self.aux_pwm_changed)
        aux_layout.addWidget(QLabel("PWM"), 0, 0)
        aux_layout.addWidget(self.aux_slider, 0, 1)
        aux_layout.addWidget(self.aux_value, 0, 2)
        layout.addWidget(aux)

        reset = QPushButton("Reset encoders")
        reset.clicked.connect(self.reset_encoders_requested)
        layout.addWidget(reset)
        layout.addStretch(1)

        self.left_slider.valueChanged.connect(lambda value: self.left_value.setText(f"{value}%"))
        self.right_slider.valueChanged.connect(lambda value: self.right_value.setText(f"{value}%"))
        self.left_slider.valueChanged.connect(self.left_pwm_changed)
        self.right_slider.valueChanged.connect(self.right_pwm_changed)

    def _make_slider(self) -> tuple[QSlider, QLabel]:
        slider = QSlider(Qt.Horizontal)
        slider.setRange(-100, 100)
        slider.setValue(0)
        slider.setTickPosition(QSlider.TicksBelow)
        slider.setTickInterval(25)
        value = QLabel("0%")
        value.setMinimumWidth(46)
        value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        return slider, value
