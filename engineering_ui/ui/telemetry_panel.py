from __future__ import annotations

import math

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QFormLayout, QGroupBox, QLabel, QSizePolicy, QVBoxLayout, QWidget

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
            "safety_distance_traveled_m": "distancia percorrida",
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
            (
                "Encoders/RPM",
                [
                    "left_encoder",
                    "right_encoder",
                    "left_rpm",
                    "right_rpm",
                    "linear_mps",
                    "safety_distance_traveled_m",
                ],
            ),
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
        self.labels["safety_distance_traveled_m"].setText(f"{state.safety_distance_traveled_m:.3f} m")
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
        self.labels["x_m"].setText(f"{state.x_m:.3f} m")
        self.labels["y_m"].setText(f"{state.y_m:.3f} m")
        self.labels["heading_rad"].setText(f"{state.heading_rad:.3f} rad")
        self.labels["encoder_heading_rad"].setText(
            f"{math.degrees(state.encoder_heading_rad):.1f} deg ({state.encoder_heading_rad:.3f} rad)"
        )
        self.labels["angular_rad_s"].setText(f"{state.angular_rad_s:.3f} rad/s")
