from __future__ import annotations

import threading
from pathlib import Path
from typing import Any

import numpy as np
import pyqtgraph.opengl as gl
from PySide6.QtCore import QTimer, Signal
from PySide6.QtGui import QMatrix4x4, QVector3D
from PySide6.QtWidgets import QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from telemetry.state import RobotState


MODEL_DIR = Path(__file__).resolve().parents[1] / "3Dmodel"
PREFERRED_MODEL_PATH = MODEL_DIR / "Aspirador_de_pista.glb"
MODEL_PITCH_OFFSET_DEG = 90.0


class Robot3DView(QWidget):
    calibrate_accel_gyro_requested = Signal()
    calibrate_yaw_drift_requested = Signal()
    calibrate_mag_requested = Signal()
    calibrate_all_requested = Signal()
    mag_ignored_requested = Signal(bool)
    _model_loaded = Signal(object)
    _model_failed = Signal(str)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setMinimumHeight(280)

        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._model_error = ""
        self._model_loading = False

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        top_bar = QWidget()
        top_layout = QHBoxLayout(top_bar)
        top_layout.setContentsMargins(0, 0, 0, 0)

        self.status = QLabel("Preparando visualizacao 3D...")
        self.status.setStyleSheet("padding: 6px; color: #d6dde6; background: #111820;")
        self.calibrate_accel_gyro_button = QPushButton("Cal acc/gyro")
        self.calibrate_accel_gyro_button.clicked.connect(self.calibrate_accel_gyro_requested)
        self.calibrate_yaw_drift_button = QPushButton("Cal yaw 20s")
        self.calibrate_yaw_drift_button.clicked.connect(self.calibrate_yaw_drift_requested)
        self.calibrate_mag_button = QPushButton("Cal mag")
        self.calibrate_mag_button.clicked.connect(self.calibrate_mag_requested)
        self.calibrate_all_button = QPushButton("Cal completa")
        self.calibrate_all_button.clicked.connect(self.calibrate_all_requested)
        self.ignore_mag_button = QPushButton("Ignorar mag")
        self.ignore_mag_button.setCheckable(True)
        self.ignore_mag_button.toggled.connect(self.mag_ignored_requested.emit)
        top_layout.addWidget(self.status, 1)
        top_layout.addWidget(self.calibrate_accel_gyro_button, 0)
        top_layout.addWidget(self.calibrate_yaw_drift_button, 0)
        top_layout.addWidget(self.calibrate_mag_button, 0)
        top_layout.addWidget(self.calibrate_all_button, 0)
        top_layout.addWidget(self.ignore_mag_button, 0)
        layout.addWidget(top_bar)

        self.view = gl.GLViewWidget()
        self.view.setBackgroundColor("#111820")
        self.view.opts["distance"] = 3.0
        self.view.opts["elevation"] = 22
        self.view.opts["azimuth"] = -42
        layout.addWidget(self.view, 1)

        grid = gl.GLGridItem()
        grid.setSize(2.5, 2.5)
        grid.setSpacing(0.25, 0.25)
        grid.translate(0, 0, -0.55)
        self.view.addItem(grid)

        self.model_path: Path | None = None
        self.model_items: list[gl.GLMeshItem] = []
        self._model_loaded.connect(self._on_model_loaded)
        self._model_failed.connect(self._on_model_failed)
        QTimer.singleShot(100, self._start_model_loader)

    def refresh(self, state: RobotState) -> None:
        self._roll = state.roll
        self._pitch = state.pitch
        self._yaw = state.yaw
        if self.ignore_mag_button.isChecked() != state.mag_ignored:
            self.ignore_mag_button.blockSignals(True)
            self.ignore_mag_button.setChecked(state.mag_ignored)
            self.ignore_mag_button.blockSignals(False)

        if self.model_items:
            matrix = QMatrix4x4()
            matrix.rotate(self._yaw, QVector3D(0, 0, 1))
            matrix.rotate(self._pitch, QVector3D(0, 1, 0))
            matrix.rotate(self._roll, QVector3D(1, 0, 0))
            matrix.rotate(MODEL_PITCH_OFFSET_DEG, QVector3D(1, 0, 0))
            for item in self.model_items:
                item.setTransform(matrix)

            self.status.setText(
                f"GLB: {self.model_path.name if self.model_path else 'modelo'} | "
                f"roll {self._roll:.1f} | pitch {self._pitch:.1f} | yaw {self._yaw:.1f} | "
                f"pitch offset {MODEL_PITCH_OFFSET_DEG:.0f}"
            )
        elif self._model_error:
            self.status.setText(self._model_error)
        elif self._model_loading:
            self.status.setText("Carregando GLB em segundo plano...")

    def _start_model_loader(self) -> None:
        if self._model_loading or self.model_items:
            return

        try:
            self.model_path = self._find_model_path()
        except Exception as exc:
            self._on_model_failed(f"Falha ao localizar GLB: {exc}")
            return

        self._model_loading = True
        self.status.setText("Carregando GLB em segundo plano...")
        loader = threading.Thread(target=self._load_model_worker, args=(self.model_path,), daemon=True)
        loader.start()

    def _load_model_worker(self, path: Path) -> None:
        try:
            payload = self._load_glb_payload(path)
        except Exception as exc:
            self._model_failed.emit(f"Falha ao carregar GLB: {exc}")
            return

        self._model_loaded.emit(payload)

    def _on_model_loaded(self, payload: object) -> None:
        self._model_loading = False
        self._model_error = ""

        meshes = list(payload) if isinstance(payload, list) else []
        total_vertices = 0
        total_faces = 0
        for vertices, faces, color in meshes:
            mesh_data = gl.MeshData(vertexes=vertices, faces=faces)
            item = gl.GLMeshItem(
                meshdata=mesh_data,
                smooth=True,
                drawEdges=False,
                color=color,
                shader="shaded",
                glOptions="opaque",
            )
            self.model_items.append(item)
            self.view.addItem(item)
            total_vertices += len(vertices)
            total_faces += len(faces)

        self.status.setText(f"GLB carregado: {total_vertices} vertices, {total_faces} faces, {len(self.model_items)} pecas")

    def _on_model_failed(self, message: str) -> None:
        self._model_loading = False
        self._model_error = message
        self.status.setText(message)

    def _find_model_path(self) -> Path:
        if PREFERRED_MODEL_PATH.exists():
            return PREFERRED_MODEL_PATH

        models = sorted(MODEL_DIR.glob("*.glb"))
        if not models:
            raise FileNotFoundError(f"nenhum .glb encontrado em {MODEL_DIR}")

        return models[0]

    @staticmethod
    def _load_glb_payload(path: Path) -> list[tuple[np.ndarray, np.ndarray, tuple[float, float, float, float]]]:
        if not path.exists():
            raise FileNotFoundError(path)

        import trimesh

        loaded = trimesh.load(str(path), force="scene")

        if isinstance(loaded, trimesh.Trimesh):
            meshes = [loaded]
        else:
            meshes = [
                mesh
                for mesh in loaded.dump(concatenate=False)
                if isinstance(mesh, trimesh.Trimesh) and len(mesh.faces) > 0
            ]

        if not meshes:
            raise ValueError("nenhuma malha encontrada no GLB")

        all_vertices = np.vstack([mesh.vertices for mesh in meshes if len(mesh.vertices) > 0])
        center = (all_vertices.min(axis=0) + all_vertices.max(axis=0)) * 0.5
        scale = np.max(np.ptp(all_vertices - center, axis=0))

        if scale > 0:
            inv_scale = 1.0 / scale
        else:
            inv_scale = 1.0

        payload = []
        for mesh in meshes:
            if len(mesh.vertices) == 0 or len(mesh.faces) == 0:
                continue

            vertices = ((mesh.vertices - center) * inv_scale).astype(np.float32)
            faces = mesh.faces.astype(np.int32)
            color = Robot3DView._mesh_color(mesh)
            payload.append((vertices, faces, color))

        if not payload:
            raise ValueError("nenhuma malha valida encontrada no GLB")

        return payload

    @staticmethod
    def _mesh_color(mesh: Any) -> tuple[float, float, float, float]:
        material = getattr(mesh.visual, "material", None)
        color = getattr(material, "main_color", None)
        if color is None:
            color = getattr(material, "baseColorFactor", None)
        if color is None:
            color = np.array([180, 180, 180, 255], dtype=np.uint8)

        color = np.asarray(color, dtype=np.float32)
        if color.max() > 1.0:
            color = color / 255.0

        if color.size < 4:
            color = np.append(color[:3], 1.0)

        color[3] = 1.0
        return tuple(float(v) for v in color[:4])
