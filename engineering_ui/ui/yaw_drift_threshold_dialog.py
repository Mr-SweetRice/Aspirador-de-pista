from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QAbstractSpinBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


class YawDriftThresholdDialog(QDialog):
    threshold_requested = Signal(float)

    def __init__(self, current_threshold: float, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Threshold drift yaw")
        self.setModal(True)

        layout = QVBoxLayout(self)

        note = QLabel("Valores de gyro Z abaixo deste limite nao serao integrados no yaw.")
        note.setWordWrap(True)
        layout.addWidget(note)

        form = QFormLayout()
        threshold_row = QWidget()
        threshold_layout = QHBoxLayout(threshold_row)
        threshold_layout.setContentsMargins(0, 0, 0, 0)
        threshold_layout.setSpacing(6)

        self.decrease_button = QPushButton("-")
        self.decrease_button.setFixedWidth(38)
        self.decrease_button.clicked.connect(lambda: self._step(-0.01))

        self.threshold_input = QDoubleSpinBox()
        self.threshold_input.setRange(0.0, 5.0)
        self.threshold_input.setDecimals(3)
        self.threshold_input.setSingleStep(0.01)
        self.threshold_input.setSuffix(" dps")
        self.threshold_input.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.threshold_input.setKeyboardTracking(False)
        self.threshold_input.setValue(max(0.0, min(5.0, current_threshold)))

        self.increase_button = QPushButton("+")
        self.increase_button.setFixedWidth(38)
        self.increase_button.clicked.connect(lambda: self._step(0.01))

        threshold_layout.addWidget(self.decrease_button)
        threshold_layout.addWidget(self.threshold_input, 1)
        threshold_layout.addWidget(self.increase_button)
        form.addRow("Threshold", threshold_row)
        layout.addLayout(form)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self._accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _accept(self) -> None:
        self.threshold_requested.emit(self.threshold_input.value())
        self.accept()

    def _step(self, delta: float) -> None:
        next_value = max(0.0, min(5.0, self.threshold_input.value() + delta))
        self.threshold_input.setValue(next_value)
        self.threshold_requested.emit(next_value)
