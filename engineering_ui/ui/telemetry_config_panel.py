from PySide6.QtCore import Signal
from PySide6.QtWidgets import QCheckBox, QGridLayout, QGroupBox, QLabel, QPushButton, QVBoxLayout, QWidget

from ble.protocol import TelemetryId


TELEMETRY_OPTIONS = (
    (TelemetryId.POSE, "Pose fundida rápida", "60 Hz", True),
    (TelemetryId.IMU_FAST, "IMU rápida", "60 Hz", True),
    (TelemetryId.LINE_FAST, "Linha rápida", "60 Hz", True),
    (TelemetryId.ODOMETRY, "Odometria completa", "~15 Hz", True),
    (TelemetryId.ENCODERS, "Encoders e RPM", "~15 Hz", True),
    (TelemetryId.BATTERY, "Bateria", "~15 Hz", True),
    (TelemetryId.CONTROL, "Estado do controle", "~15 Hz", True),
    (TelemetryId.IMU, "IMU completa", "~15 Hz", True),
    (TelemetryId.LINE, "Linha completa", "até 30 Hz", True),
    (TelemetryId.SAFETY, "Segurança", "~15 Hz", True),
    (TelemetryId.SYSTEM, "CPU e sistema", "~15 Hz", True),
    (TelemetryId.RGB_LED, "LED RGB", "~15 Hz", True),
    (TelemetryId.PORTAL, "Sensor de portal", "~15 Hz", True),
)


class TelemetryConfigPanel(QWidget):
    changed = Signal(int, bool)

    def __init__(self, settings, parent=None):
        super().__init__(parent)
        self.settings = settings
        self.checks = {}
        root = QVBoxLayout(self)
        root.addWidget(QLabel(
            "Habilite somente os dados necessários. Status de comandos e transferências de mapas "
            "permanecem ativos para preservar a comunicação."
        ))
        group = QGroupBox("Pacotes periódicos")
        grid = QGridLayout(group)
        for row, (message_id, label, rate, default) in enumerate(TELEMETRY_OPTIONS):
            check = QCheckBox(label)
            check.setChecked(settings.value(f"telemetry/{int(message_id):02x}", default, type=bool))
            check.toggled.connect(lambda enabled, mid=int(message_id): self._change(mid, enabled))
            self.checks[int(message_id)] = check
            grid.addWidget(check, row, 0)
            grid.addWidget(QLabel(rate), row, 1)
        root.addWidget(group)
        buttons = QGridLayout()
        enable_all = QPushButton("Habilitar todos")
        disable_heavy = QPushButton("Modo leve")
        enable_all.clicked.connect(lambda: self.set_all(True))
        disable_heavy.clicked.connect(self.set_light_mode)
        buttons.addWidget(enable_all, 0, 0)
        buttons.addWidget(disable_heavy, 0, 1)
        root.addLayout(buttons)
        root.addStretch(1)

    def _change(self, message_id, enabled):
        self.settings.setValue(f"telemetry/{message_id:02x}", enabled)
        self.changed.emit(message_id, enabled)

    def set_all(self, enabled):
        for check in self.checks.values():
            check.setChecked(enabled)

    def set_light_mode(self):
        keep = {int(TelemetryId.POSE), int(TelemetryId.ODOMETRY), int(TelemetryId.CONTROL),
                int(TelemetryId.BATTERY), int(TelemetryId.SAFETY)}
        for message_id, check in self.checks.items():
            check.setChecked(message_id in keep)

    def selections(self):
        return [(message_id, check.isChecked()) for message_id, check in self.checks.items()]
