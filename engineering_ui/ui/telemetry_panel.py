from __future__ import annotations

import math

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QFormLayout, QGroupBox, QLabel, QSizePolicy, QVBoxLayout, QWidget

from ble.protocol import MAG_HEADING_MODE_LABELS
from telemetry.state import RobotState


class TelemetryPanel(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)
        self.setMinimumWidth(360)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.MinimumExpanding)
        self.labels: dict[str, QLabel] = {}
        field_names = {
            "linear_mps": "velocidade",
            "angular_rad_s": "angular rad/s",
            "encoder_heading_rad": "angulo encoder",
            "control_loop_hz": "control_task",
            "track_odometry_loop_hz": "track_odometry",
            "race_plan_loop_hz": "race_plan",
            "line_sensor_loop_hz": "line_sensor",
            "imu_loop_hz": "imu_task",
            "cpu0_percent": "Comms core 0",
            "cpu1_percent": "Controle core 1",
        }

        for title, keys in [
            ("Encoders/RPM", ["left_encoder", "right_encoder", "left_rpm", "right_rpm", "linear_mps"]),
            ("Bateria", ["battery_raw", "battery_v", "battery_percent"]),
            ("Uso de CPU", ["cpu0_percent", "cpu1_percent"]),
            (
                "Tasks",
                [
                    "control_loop_hz",
                    "track_odometry_loop_hz",
                    "race_plan_loop_hz",
                    "line_sensor_loop_hz",
                    "imu_loop_hz",
                ],
            ),
            (
                "IMU",
                [
                    "roll",
                    "pitch",
                    "yaw",
                    "mag_yaw",
                    "mag_yaw_xy",
                    "mag_yaw_xz",
                    "mag_yaw_yz",
                    "mag_yaw_error",
                    "mag_norm",
                    "mag_ignorado",
                    "mag_heading_mode",
                    "mag_filter_gain",
                    "quat",
                    "accel",
                    "gyro",
                    "mag",
                    "calibracao_acc_gyro",
                    "calibracao_yaw",
                    "calibracao_mag",
                ],
            ),
            ("Odometria", ["x_m", "y_m", "heading_rad", "encoder_heading_rad", "angular_rad_s"]),
        ]:
            group = QGroupBox(title)
            form = QFormLayout(group)
            form.setContentsMargins(16, 20, 16, 14)
            form.setHorizontalSpacing(18)
            form.setVerticalSpacing(8)
            group.setMinimumHeight(58 + len(keys) * 28)
            group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Minimum)
            for key in keys:
                label = QLabel("-")
                label.setMinimumHeight(22)
                label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
                label.setTextInteractionFlags(label.textInteractionFlags() | Qt.TextSelectableByMouse)
                self.labels[key] = label
                form.addRow(field_names.get(key, key), label)
            layout.addWidget(group)

        layout.addStretch(1)

    def refresh(self, state: RobotState) -> None:
        self.labels["left_encoder"].setText(str(state.left_encoder))
        self.labels["right_encoder"].setText(str(state.right_encoder))
        self.labels["left_rpm"].setText(f"{state.left_rpm:.1f}")
        self.labels["right_rpm"].setText(f"{state.right_rpm:.1f}")
        self.labels["linear_mps"].setText(f"{state.linear_mps:.3f} m/s")
        self.labels["battery_raw"].setText(str(state.battery_raw))
        self.labels["battery_v"].setText(f"{state.battery_v:.2f} V")
        self.labels["battery_percent"].setText(f"{state.battery_percent:.1f}%")
        self.labels["cpu0_percent"].setText(f"{state.cpu0_percent:.0f}%")
        self.labels["cpu1_percent"].setText(f"{state.cpu1_percent:.0f}%")
        self.labels["control_loop_hz"].setText(f"{state.control_loop_hz:.0f} Hz")
        self.labels["track_odometry_loop_hz"].setText(f"{state.track_odometry_loop_hz:.0f} Hz")
        self.labels["race_plan_loop_hz"].setText(f"{state.race_plan_loop_hz:.0f} Hz")
        self.labels["line_sensor_loop_hz"].setText(f"{state.line_sensor_loop_hz:.0f} Hz")
        self.labels["imu_loop_hz"].setText(f"{state.imu_loop_hz:.0f} Hz")
        self.labels["roll"].setText(f"{state.roll:.1f} deg")
        self.labels["pitch"].setText(f"{state.pitch:.1f} deg")
        self.labels["yaw"].setText(f"{state.yaw:.1f} deg")
        self.labels["mag_yaw"].setText(f"{state.mag_yaw:.1f} deg")
        self.labels["mag_yaw_xy"].setText(f"{state.mag_yaw_xy:.1f} deg")
        self.labels["mag_yaw_xz"].setText(f"{state.mag_yaw_xz:.1f} deg")
        self.labels["mag_yaw_yz"].setText(f"{state.mag_yaw_yz:.1f} deg")
        self.labels["mag_yaw_error"].setText(f"{state.mag_yaw_error:.1f} deg")
        self.labels["mag_norm"].setText(f"{state.mag_norm:.2f} uT")
        self.labels["mag_ignorado"].setText("sim" if state.mag_ignored else "nao")
        self.labels["mag_heading_mode"].setText(MAG_HEADING_MODE_LABELS.get(state.mag_heading_mode, str(state.mag_heading_mode)))
        self.labels["mag_filter_gain"].setText(f"{state.mag_filter_gain:.4f}")
        self.labels["quat"].setText(", ".join(f"{v:.3f}" for v in state.quaternion))
        self.labels["accel"].setText(", ".join(f"{v:.2f}" for v in state.accel))
        self.labels["gyro"].setText(", ".join(f"{v:.2f}" for v in state.gyro))
        self.labels["mag"].setText(", ".join(f"{v:.2f}" for v in state.mag))
        if state.accel_gyro_calibrating:
            accel_gyro_calibration = "calibrando"
        elif state.accel_gyro_calibrated:
            accel_gyro_calibration = "calibrado"
        else:
            accel_gyro_calibration = "pendente"
        self.labels["calibracao_acc_gyro"].setText(accel_gyro_calibration)
        if state.yaw_drift_calibrating:
            yaw_calibration = "calibrando"
        elif state.yaw_drift_calibrated:
            yaw_calibration = "calibrado"
        else:
            yaw_calibration = "pendente"
        self.labels["calibracao_yaw"].setText(yaw_calibration)
        if state.mag_calibrating:
            calibration = "calibrando"
        elif state.mag_calibrated:
            calibration = "calibrado"
        else:
            calibration = "pendente"
        self.labels["calibracao_mag"].setText(calibration)
        self.labels["x_m"].setText(f"{state.x_m:.3f} m")
        self.labels["y_m"].setText(f"{state.y_m:.3f} m")
        self.labels["heading_rad"].setText(f"{state.heading_rad:.3f} rad")
        self.labels["encoder_heading_rad"].setText(
            f"{math.degrees(state.encoder_heading_rad):.1f} deg ({state.encoder_heading_rad:.3f} rad)"
        )
        self.labels["angular_rad_s"].setText(f"{state.angular_rad_s:.3f} rad/s")
