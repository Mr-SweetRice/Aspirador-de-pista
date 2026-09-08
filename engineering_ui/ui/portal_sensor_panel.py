from collections import deque
from time import monotonic

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from telemetry.state import RobotState


class PortalSensorPanel(QWidget):
    config_requested = Signal(bool, int, int, int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._updating = False
        self._packet_times: deque[float] = deque(maxlen=120)
        self._packet_count = 0
        self._last_sample_at: float | None = None

        root = QVBoxLayout(self)
        self.overall_status = QLabel("AGUARDANDO TELEMETRIA DIGITAL")
        self.overall_status.setWordWrap(True)
        self.overall_status.setMinimumHeight(48)
        root.addWidget(self.overall_status)

        settings = QGroupBox("Contagem digital de portal")
        form = QFormLayout(settings)
        self.enabled = QCheckBox("Ativar contagem durante a corrida")
        self.speed = QSpinBox()
        self.speed.setRange(0, 100)
        self.speed.setValue(20)
        self.speed.setSuffix(" %")
        self.delay = QSpinBox()
        self.delay.setRange(0, 10000)
        self.delay.setValue(500)
        self.delay.setSingleStep(100)
        self.delay.setSuffix(" ms")
        form.addRow(self.enabled)
        form.addRow("Velocidade ao iniciar parada", self.speed)
        form.addRow("Tempo ate parar", self.delay)
        root.addWidget(settings)

        status = QGroupBox("Estado da entrada digital")
        state = QFormLayout(status)
        self.input_ready = QLabel("-")
        self.gpio1 = QLabel("-")
        self.gpio1_events = QLabel("-")
        self.detected = QLabel("-")
        self.count = QLabel("-")
        self.stopping = QLabel("-")
        state.addRow("Ligacao", QLabel("GPIO1/OUT do sensor → GPIO12 do ESP32"))
        state.addRow("Entrada digital", self.input_ready)
        state.addRow("Nivel GPIO12", self.gpio1)
        state.addRow("Pulsos recebidos", self.gpio1_events)
        state.addRow("Portal ativo", self.detected)
        state.addRow("Contagem", self.count)
        state.addRow("Parada em curso", self.stopping)
        root.addWidget(status)

        debug = QGroupBox("Debug digital")
        debug_form = QFormLayout(debug)
        self.packet_count = QLabel("0")
        self.update_rate = QLabel("-")
        self.sample_age = QLabel("sem telemetria")
        self.raw_flags = QLabel("-")
        debug_form.addRow("Pacotes recebidos", self.packet_count)
        debug_form.addRow("Taxa de atualizacao", self.update_rate)
        debug_form.addRow("Idade do pacote", self.sample_age)
        debug_form.addRow("Flags", self.raw_flags)
        root.addWidget(debug)

        actions = QHBoxLayout()
        self.reset_debug = QPushButton("Zerar estatisticas")
        actions.addStretch(1)
        actions.addWidget(self.reset_debug)
        root.addLayout(actions)
        root.addStretch(1)

        self.enabled.toggled.connect(self._emit)
        self.speed.editingFinished.connect(self._emit)
        self.delay.editingFinished.connect(self._emit)
        self.reset_debug.clicked.connect(self._reset_debug_stats)

    def _emit(self, *_):
        if not self._updating:
            # O campo de limiar permanece no protocolo apenas por compatibilidade.
            self.config_requested.emit(
                self.enabled.isChecked(), 300, self.speed.value(), self.delay.value()
            )

    def record_sample(self, robot: RobotState) -> None:
        now = monotonic()
        self._last_sample_at = now
        self._packet_times.append(now)
        self._packet_count += 1

    def _reset_debug_stats(self) -> None:
        self._packet_times.clear()
        self._packet_count = 0
        self._last_sample_at = None

    def _set_banner(self, text: str, color: str) -> None:
        self.overall_status.setText(text)
        self.overall_status.setStyleSheet(
            f"font-size: 16px; font-weight: bold; padding: 9px; background: {color}; "
            "color: white; border-radius: 4px;"
        )

    def refresh(self, robot: RobotState):
        self._updating = True
        self.enabled.setChecked(robot.portal_enabled)
        if not self.speed.hasFocus():
            self.speed.setValue(robot.portal_stop_speed_percent)
        if not self.delay.hasFocus():
            self.delay.setValue(robot.portal_stop_delay_ms)
        self._updating = False

        if not robot.connected:
            self._set_banner("DESCONECTADO — SEM TELEMETRIA", "#555555")
        elif self._last_sample_at is None:
            self._set_banner("BLE CONECTADO — AGUARDANDO PACOTE DIGITAL", "#8a6500")
        elif not robot.portal_gpio1_available:
            self._set_banner("FIRMWARE ANTIGO — ESTADO GPIO1 INDISPONIVEL", "#a32626")
        elif robot.portal_gpio1_active:
            self._set_banner("PORTAL ATIVO — GPIO12 EM NIVEL BAIXO", "#b05a00")
        else:
            self._set_banner("ENTRADA DIGITAL OK — AGUARDANDO PULSO", "#19733b")

        self.input_ready.setText("CONFIGURADA" if robot.portal_sensor_ok else "SEM TELEMETRIA")
        if robot.portal_gpio1_available:
            self.gpio1.setText("BAIXO — ativo" if robot.portal_gpio1_active else "ALTO — inativo")
            self.gpio1_events.setText(str(robot.portal_gpio1_events))
        else:
            self.gpio1.setText("indisponivel no firmware atual")
            self.gpio1_events.setText("-")
        self.detected.setText("SIM" if robot.portal_detected else "nao")
        self.count.setText(f"{robot.portal_count} / 2")
        self.stopping.setText("SIM" if robot.portal_stopping else "nao")
        self.packet_count.setText(str(self._packet_count))

        now = monotonic()
        while self._packet_times and now - self._packet_times[0] > 2.0:
            self._packet_times.popleft()
        if len(self._packet_times) >= 2:
            elapsed = self._packet_times[-1] - self._packet_times[0]
            rate = (len(self._packet_times) - 1) / elapsed if elapsed > 0 else 0.0
            self.update_rate.setText(f"{rate:.1f} Hz")
        else:
            self.update_rate.setText("-")
        if self._last_sample_at is None:
            self.sample_age.setText("sem telemetria")
        else:
            self.sample_age.setText(f"{int((now - self._last_sample_at) * 1000)} ms")
        self.raw_flags.setText(
            f"gpio={int(robot.portal_gpio1_active)} pulsos={robot.portal_gpio1_events} "
            f"portal={int(robot.portal_detected)} contagem={robot.portal_count} "
            f"habilitado={int(robot.portal_enabled)}"
        )
