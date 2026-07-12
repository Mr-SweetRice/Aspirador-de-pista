from __future__ import annotations

import csv
from pathlib import Path

from telemetry.state import RobotState


class TelemetryCsvLogger:
    def __init__(self, path: Path) -> None:
        self.path = path
        self._file = path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(
            [
                "sample",
                "left_encoder",
                "right_encoder",
                "left_rpm",
                "right_rpm",
                "battery_v",
                "battery_percent",
                "roll",
                "pitch",
                "yaw",
                "x_m",
                "y_m",
            ]
        )

    def write(self, state: RobotState) -> None:
        self._writer.writerow(
            [
                state.samples,
                state.left_encoder,
                state.right_encoder,
                f"{state.left_rpm:.3f}",
                f"{state.right_rpm:.3f}",
                f"{state.battery_v:.3f}",
                f"{state.battery_percent:.3f}",
                f"{state.roll:.3f}",
                f"{state.pitch:.3f}",
                f"{state.yaw:.3f}",
                f"{state.x_m:.4f}",
                f"{state.y_m:.4f}",
            ]
        )

    def close(self) -> None:
        self._file.close()
