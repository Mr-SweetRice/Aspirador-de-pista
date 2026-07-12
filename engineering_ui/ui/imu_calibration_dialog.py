from __future__ import annotations

from PySide6.QtCore import QTimer, Signal
from PySide6.QtWidgets import QDialog, QLabel, QPushButton, QProgressBar, QVBoxLayout


class ImuCalibrationDialog(QDialog):
    start_requested = Signal(int)

    def __init__(self, mode: str = "all", duration_s: int = 30, parent=None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Calibracao IMU")
        self.mode = mode
        self.duration_s = duration_s
        self.remaining_s = duration_s

        layout = QVBoxLayout(self)
        self.instructions = QLabel(self._instructions())
        self.instructions.setWordWrap(True)
        self.progress = QProgressBar()
        self.progress.setRange(0, duration_s)
        self.progress.setValue(0)
        self.step_label = QLabel("Pronto para iniciar.")
        self.start_button = QPushButton("Iniciar calibracao")
        self.close_button = QPushButton("Fechar")

        layout.addWidget(self.instructions)
        layout.addWidget(self.step_label)
        layout.addWidget(self.progress)
        layout.addWidget(self.start_button)
        layout.addWidget(self.close_button)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.start_button.clicked.connect(self._start)
        self.close_button.clicked.connect(self.accept)

    def _start(self) -> None:
        self.remaining_s = self.duration_s
        self.progress.setValue(0)
        self.start_button.setEnabled(False)
        self.step_label.setText(self._step_text(0))
        self.start_requested.emit(self.duration_s)
        self.timer.start(1000)

    def _tick(self) -> None:
        self.remaining_s -= 1
        elapsed = self.duration_s - self.remaining_s
        self.progress.setValue(elapsed)

        if self.remaining_s > 0:
            self.step_label.setText(self._step_text(elapsed))
        else:
            self.timer.stop()
            self.progress.setValue(self.duration_s)
            self.step_label.setText("Calibracao concluida. Mantenha o robo parado por alguns segundos.")
            self.start_button.setEnabled(True)

    def _instructions(self) -> str:
        if self.mode == "accel_gyro":
            return (
                "Calibracao accel/gyro do MPU-9250.\n\n"
                "1. Coloque o robo em uma superficie firme e nivelada.\n"
                "2. Desligue motores e mantenha o robo totalmente parado.\n"
                "3. Clique em Iniciar e nao toque no robo ate concluir."
            )
        if self.mode == "yaw_drift":
            return (
                "Calibracao do drift de yaw do giroscopio.\n\n"
                "1. Desligue os motores.\n"
                "2. Deixe o robo totalmente parado por 20 segundos.\n"
                "3. O bias do gyro Z sera medido e subtraido das leituras."
            )
        if self.mode == "mag":
            return (
                "Calibracao do magnetometro AK8963.\n\n"
                "1. Afaste o robo de imas, motores ligados, cabos de alta corrente e estruturas metalicas.\n"
                "2. Clique em Iniciar.\n"
                "3. Gire lentamente o robo em todas as orientacoes, cobrindo X, Y e Z.\n"
                "4. Use movimentos em oito e rotacoes completas durante todo o tempo."
            )
        return (
            "Calibracao completa do MPU-9250.\n\n"
            "1. Afaste o robo de imas, motores ligados, cabos de alta corrente e estruturas metalicas.\n"
            "2. Clique em Iniciar.\n"
            "3. Nos primeiros segundos, mantenha o robo parado e nivelado.\n"
            "4. Depois, gire lentamente em todas as orientacoes, como um movimento em oito."
        )

    def _step_text(self, elapsed_s: int) -> str:
        if self.mode == "accel_gyro":
            return "Passo atual: manter parado e nivelado."
        if self.mode == "yaw_drift":
            return "Passo atual: manter o robo totalmente parado."
        if self.mode == "mag":
            if elapsed_s < self.duration_s * 0.33:
                return "Passo atual: incline para frente e para tras."
            if elapsed_s < self.duration_s * 0.66:
                return "Passo atual: incline para esquerda e direita."
            return "Passo atual: gire 360 graus no eixo vertical."

        if elapsed_s < 6:
            return "Passo atual: parado e nivelado para gyro/acelerometro."
        if elapsed_s < self.duration_s * 0.50:
            return "Passo atual: incline para frente e para tras."
        if elapsed_s < self.duration_s * 0.75:
            return "Passo atual: incline para esquerda e direita."
        return "Passo atual: gire 360 graus no eixo vertical."
