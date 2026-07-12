from __future__ import annotations

import math

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QProgressBar,
    QPushButton,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from ble.protocol import ODOMETRY_SOURCE_FUSED, ODOMETRY_SOURCE_LABELS
from telemetry.state import RobotState
from ui.robot_marker import GpsRobotMarker


class EditableMapPlot(pg.PlotWidget):
    def __init__(self, owner: "MapView") -> None:
        super().__init__(title="Odometria 2D")
        self.owner = owner

    def mousePressEvent(self, event) -> None:
        if self.owner.handle_plot_mouse_press(event):
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if self.owner.handle_plot_mouse_move(event):
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        if self.owner.handle_plot_mouse_release(event):
            event.accept()
            return
        super().mouseReleaseEvent(event)


class MapView(QWidget):
    save_map_requested = Signal(str, list)
    record_start_requested = Signal(str)
    record_stop_requested = Signal()
    record_save_requested = Signal(str)
    map_list_requested = Signal()
    map_load_requested = Signal(int)
    map_delete_requested = Signal(int, str)
    reset_yaw_requested = Signal()
    RECORD_MIN_STEP_M = 0.02
    RECORD_MAX_STEP_M = 0.30

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._recording = False
        self._editing = False
        self._selected_index: int | None = None
        self._recorded_points: list[tuple[float, float]] = []
        self._loaded_points: list[tuple[float, float]] = []
        self._available_maps: list[dict] = []
        self._dragging_point = False
        self._loaded_distance_m = 0.0
        self._loading_map = False
        self._loading_received = 0
        self._rejected_record_points = 0
        self._record_distance_m = 0.0
        self._syncing_record_button = False
        self._loaded_index_labels: list[pg.TextItem] = []
        self._recorded_index_labels: list[pg.TextItem] = []

        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(8)

        self.menu_toggle = QToolButton()
        self.menu_toggle.setText("Menu")
        self.menu_toggle.setCheckable(True)
        self.menu_toggle.setChecked(True)
        self.menu_toggle.clicked.connect(self._toggle_sidebar)
        root.addWidget(self.menu_toggle, 0)

        self.sidebar = QWidget()
        self.sidebar.setFixedWidth(245)
        menu = QVBoxLayout(self.sidebar)
        menu.setContentsMargins(8, 8, 8, 8)
        menu.setSpacing(8)

        self.map_name = QLineEdit("mapa_1")
        self.record_source_combo = QComboBox()
        self.record_source_combo.addItem(ODOMETRY_SOURCE_LABELS[ODOMETRY_SOURCE_FUSED], ODOMETRY_SOURCE_FUSED)
        self.record_source_combo.setEnabled(False)
        self.record_source_combo.setToolTip("A gravacao de pista usa somente odometria combinada com IMU disponivel")
        self.record_button = QPushButton("Start record")
        self.record_button.setCheckable(True)
        self.record_button.toggled.connect(self._set_recording)
        self.save_record_button = QPushButton("Salvar gravacao")
        self.save_record_button.clicked.connect(self._save_recording)

        form = QFormLayout()
        form.addRow("Nome", self.map_name)
        form.addRow("Salvar odometria", self.record_source_combo)
        menu.addLayout(form)
        menu.addWidget(self.record_button)
        menu.addWidget(self.save_record_button)

        menu.addSpacing(10)
        menu.addWidget(QLabel("Mapas na memoria"))
        self.map_combo = QComboBox()
        self.refresh_maps_button = QPushButton("Atualizar lista")
        self.load_map_button = QPushButton("Carregar mapa")
        self.delete_map_button = QPushButton("Apagar mapa")
        self.delete_map_button.setObjectName("stopButton")
        self.load_progress = QProgressBar()
        self.load_progress.setRange(0, 100)
        self.load_progress.setValue(0)
        self.load_progress.setTextVisible(True)
        self.load_progress.hide()
        self.show_loaded_map = QCheckBox("Exibir no mapa")
        self.show_loaded_map.setChecked(True)
        self.show_index_labels = QCheckBox("Exibir indices")
        self.show_index_labels.setChecked(True)
        self.edit_button = QPushButton("Editar mapa")
        self.edit_button.setCheckable(True)
        self.save_edit_button = QPushButton("Salvar edicao")

        self.refresh_maps_button.clicked.connect(self.map_list_requested)
        self.load_map_button.clicked.connect(self._load_selected_map)
        self.delete_map_button.clicked.connect(self._delete_selected_map)
        self.show_loaded_map.toggled.connect(self._draw_loaded_map)
        self.show_index_labels.toggled.connect(self._redraw_index_labels)
        self.edit_button.toggled.connect(self._set_editing)
        self.save_edit_button.clicked.connect(self._save_edit)

        menu.addWidget(self.map_combo)
        menu.addWidget(self.refresh_maps_button)
        menu.addWidget(self.load_map_button)
        menu.addWidget(self.delete_map_button)
        menu.addWidget(self.load_progress)
        menu.addWidget(self.show_loaded_map)
        menu.addWidget(self.show_index_labels)
        menu.addWidget(self.edit_button)
        menu.addWidget(self.save_edit_button)

        menu.addSpacing(10)
        self.reset_yaw_button = QPushButton("Resetar angulo")
        self.reset_yaw_button.clicked.connect(self.reset_yaw_requested)
        menu.addWidget(self.reset_yaw_button)

        menu.addStretch(1)
        self.record_status = QLabel("0 pontos")
        self.map_distance_status = QLabel("pista: 0.000 m")
        menu.addWidget(self.record_status)
        menu.addWidget(self.map_distance_status)
        root.addWidget(self.sidebar, 0)

        self.plot = EditableMapPlot(self)
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.addLegend(offset=(10, 10))
        self.fused_path = self.plot.plot(
            pen=pg.mkPen("#27ae60", width=2),
            symbol="o",
            symbolSize=4,
            symbolBrush="#27ae60",
            symbolPen=None,
            name="Combinado",
        )
        self.recorded_path = self.plot.plot(pen=pg.mkPen("#27ae60", width=2))
        self.loaded_path = self.plot.plot(pen=pg.mkPen("#f2c94c", width=2))
        self.loaded_points_item = pg.ScatterPlotItem(size=9, brush="#f2c94c", pen=pg.mkPen("#111820", width=1))
        self.plot.addItem(self.loaded_points_item)
        self.robot_marker = GpsRobotMarker()
        self.robot_marker.add_to(self.plot)
        root.addWidget(self.plot, 1)

    def set_available_maps(self, maps: list[dict]) -> None:
        current_data = self.map_combo.currentData()
        current_slot = current_data.get("slot") if current_data else None
        self._available_maps = maps
        self.map_combo.clear()
        for item in maps:
            distance = float(item.get("distance_m", 0.0))
            self.map_combo.addItem(f"{item['name']} ({item['point_count']} pts, {distance:.2f} m)", item)
        if current_slot is not None:
            for index in range(self.map_combo.count()):
                data = self.map_combo.itemData(index)
                if data and data.get("slot") == current_slot:
                    self.map_combo.setCurrentIndex(index)
                    break

    def start_loaded_map(self, slot: int, total_points: int) -> None:
        self._loaded_points = [(0.0, 0.0)] * total_points
        self._selected_index = None
        self._loaded_distance_m = 0.0
        self._loading_map = True
        self._loading_received = 0
        self.loaded_path.clear()
        self.loaded_points_item.clear()
        self._clear_loaded_index_labels()
        self.load_progress.setRange(0, max(1, total_points))
        self.load_progress.setValue(0)
        self.load_progress.setFormat("Carregando mapa: 0%")
        self.load_progress.show()
        for index in range(self.map_combo.count()):
            data = self.map_combo.itemData(index)
            if data and data.get("slot") == slot:
                self.map_combo.setCurrentIndex(index)
                self.map_name.setText(data.get("name", self.map_name.text()))
                self._loaded_distance_m = float(data.get("distance_m", 0.0))
                self._update_distance_status()
                break

    def set_loaded_chunk(self, offset: int, points: list[tuple[float, float]]) -> None:
        for index, point in enumerate(points):
            target = offset + index
            if 0 <= target < len(self._loaded_points):
                self._loaded_points[target] = point
        self._loading_received = min(len(self._loaded_points), max(self._loading_received, offset + len(points)))
        if self._loading_map:
            self.load_progress.setValue(self._loading_received)
            total = max(1, len(self._loaded_points))
            percent = int((self._loading_received * 100) / total)
            self.load_progress.setFormat(f"Carregando mapa: {percent}%")
            return
        self._draw_loaded_map()

    def finish_loaded_map(self, slot: int) -> None:
        del slot
        self._loading_map = False
        self.load_progress.hide()
        self._draw_loaded_map()

    def refresh(self, state: RobotState) -> None:
        if state.fused_path_history:
            path = self._polyline_array(state.fused_path_history, self.RECORD_MAX_STEP_M)
            self.fused_path.setData(path[:, 0], path[:, 1])
        self.robot_marker.set_pose(state.fused_x_m, state.fused_y_m, state.fused_heading_rad)

    def _toggle_sidebar(self) -> None:
        self.sidebar.setVisible(self.menu_toggle.isChecked())

    def _set_recording(self, enabled: bool) -> None:
        if self._syncing_record_button:
            return
        self._recording = enabled
        self.record_button.setText("Stop record" if enabled else "Start record")
        if enabled:
            self._recorded_points = []
            self._rejected_record_points = 0
            self._record_distance_m = 0.0
            self._draw_recording()
            self._update_distance_status()
            self.record_start_requested.emit(self.map_name.text())
        else:
            self.record_stop_requested.emit()

    def _set_editing(self, enabled: bool) -> None:
        self._editing = enabled
        self.edit_button.setText("Editando" if enabled else "Editar mapa")
        self._selected_index = None
        self._dragging_point = False
        self._draw_loaded_map()

    def _save_recording(self) -> None:
        self.record_save_requested.emit(self.map_name.text())

    def _save_edit(self) -> None:
        if self._loaded_points:
            self.save_map_requested.emit(self.map_name.text(), list(self._loaded_points))

    def _load_selected_map(self) -> None:
        data = self.map_combo.currentData()
        if data:
            self.map_load_requested.emit(int(data["slot"]))

    def _delete_selected_map(self) -> None:
        data = self.map_combo.currentData()
        if data:
            self.map_delete_requested.emit(int(data["slot"]), str(data.get("name", "")))

    def clear_loaded_map(self, slot: int | None = None) -> None:
        if slot is not None:
            current_data = self.map_combo.currentData()
            current_slot = current_data.get("slot") if current_data else None
            if current_slot is not None and int(current_slot) != int(slot):
                return
        self._loaded_points = []
        self._selected_index = None
        self._loaded_distance_m = 0.0
        self._loading_map = False
        self.loaded_path.clear()
        self.loaded_points_item.clear()
        self._clear_loaded_index_labels()
        self.load_progress.hide()
        self._update_distance_status()

    def _draw_recording(self) -> None:
        distance = self._record_distance_m if self._record_distance_m > 0.0 else self._path_distance(self._recorded_points)
        gap_indexes = self._gap_indexes(self._recorded_points, self.RECORD_MAX_STEP_M)
        rejected = f" | {self._rejected_record_points} saltos ignorados" if self._rejected_record_points else ""
        gaps = f" | {len(gap_indexes)} gaps" if gap_indexes else ""
        active = "gravando" if self._recording else "parado"
        waiting = " | aguardando odometria/IMU" if self._recording and not self._recorded_points else ""
        self.record_status.setText(f"{active}: {len(self._recorded_points)} pontos{waiting}{rejected}{gaps}")
        self.map_distance_status.setText(f"gravacao: {distance:.3f} m")
        if not self._recorded_points:
            self.recorded_path.clear()
            self._clear_recorded_index_labels()
            return
        points = self._polyline_array(self._recorded_points, self.RECORD_MAX_STEP_M)
        self.recorded_path.setData(points[:, 0], points[:, 1])
        self._draw_index_labels(self._recorded_points, self._recorded_index_labels, "#27ae60", set(gap_indexes))

    def set_recorded_chunk(
        self,
        total_points: int,
        offset: int,
        points: list[tuple[float, float]],
        active: bool,
        rejected_points: int,
        distance_m: float,
    ) -> None:
        if offset == 0 and total_points < len(self._recorded_points):
            self._recorded_points = []
        while len(self._recorded_points) < offset:
            self._recorded_points.append((math.nan, math.nan))
        for index, point in enumerate(points):
            target = offset + index
            if target < len(self._recorded_points):
                self._recorded_points[target] = point
            else:
                self._recorded_points.append(point)
        if not active and len(self._recorded_points) > total_points:
            self._recorded_points = self._recorded_points[:total_points]
        self._rejected_record_points = int(rejected_points)
        self._record_distance_m = float(distance_m)
        self._set_record_button_state(active)
        self._draw_recording()

    def _set_record_button_state(self, active: bool) -> None:
        if self._recording == active and self.record_button.isChecked() == active:
            return
        self._syncing_record_button = True
        self._recording = active
        self.record_button.setChecked(active)
        self.record_button.setText("Stop record" if active else "Start record")
        self._syncing_record_button = False

    def _draw_loaded_map(self) -> None:
        visible = self.show_loaded_map.isChecked() and bool(self._loaded_points)
        if not visible:
            self.loaded_path.clear()
            self.loaded_points_item.clear()
            self._clear_loaded_index_labels()
            self._update_distance_status()
            return

        points = self._polyline_array(self._loaded_points, self.RECORD_MAX_STEP_M)
        self.loaded_path.setData(points[:, 0], points[:, 1])
        gap_indexes = set(self._gap_indexes(self._loaded_points, self.RECORD_MAX_STEP_M))
        spots = []
        for index, point in enumerate(self._loaded_points):
            if index == self._selected_index:
                brush = "#56ccf2"
            elif index in gap_indexes:
                brush = "#eb5757"
            else:
                brush = "#f2c94c"
            spots.append({"pos": point, "data": index, "brush": brush})
        self.loaded_points_item.setData(spots)
        self._draw_index_labels(self._loaded_points, self._loaded_index_labels, "#f2c94c", gap_indexes)
        self._update_distance_status()

    def handle_plot_mouse_press(self, event) -> bool:
        if not self._editing or not self._loaded_points or event.button() != Qt.LeftButton:
            return False

        scene_pos = self.plot.mapToScene(event.position().toPoint())
        index = self._nearest_point_index(scene_pos)
        if index is None:
            return False

        self._selected_index = index
        self._dragging_point = True
        self._move_selected_point(scene_pos)
        self._draw_loaded_map()
        return True

    def handle_plot_mouse_move(self, event) -> bool:
        if not self._editing or not self._dragging_point or self._selected_index is None:
            return False
        self._move_selected_point(self.plot.mapToScene(event.position().toPoint()))
        self._draw_loaded_map()
        return True

    def handle_plot_mouse_release(self, event) -> bool:
        if event.button() != Qt.LeftButton or not self._dragging_point:
            return False
        self._dragging_point = False
        return True

    def _move_selected_point(self, scene_pos) -> None:
        if self._selected_index is None:
            return
        point = self.plot.plotItem.vb.mapSceneToView(scene_pos)
        self._loaded_points[self._selected_index] = (float(point.x()), float(point.y()))
        self._loaded_distance_m = self._path_distance(self._loaded_points)

    def _nearest_point_index(self, scene_pos) -> int | None:
        view_box = self.plot.plotItem.vb
        best_index: int | None = None
        best_distance = 14.0

        for index, point in enumerate(self._loaded_points):
            point_scene = view_box.mapViewToScene(pg.Point(point[0], point[1]))
            distance = math.hypot(point_scene.x() - scene_pos.x(), point_scene.y() - scene_pos.y())
            if distance < best_distance:
                best_distance = distance
                best_index = index
        return best_index

    @staticmethod
    def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _update_distance_status(self) -> None:
        if self._recording:
            return
        if self._loaded_points:
            distance = self._loaded_distance_m if self._loaded_distance_m > 0.0 else self._path_distance(self._loaded_points)
            gap_count = len(self._gap_indexes(self._loaded_points, self.RECORD_MAX_STEP_M))
            suffix = f" | {gap_count} gap(s)" if gap_count else ""
            self.map_distance_status.setText(f"pista: {distance:.3f} m{suffix}")
        else:
            self.map_distance_status.setText("pista: 0.000 m")

    def _path_distance(self, points: list[tuple[float, float]]) -> float:
        if len(points) < 2:
            return 0.0
        return sum(self._distance(points[index - 1], points[index]) for index in range(1, len(points)))

    def _redraw_index_labels(self) -> None:
        self._clear_recorded_index_labels()
        self._clear_loaded_index_labels()
        self._draw_recording()
        self._draw_loaded_map()

    def _draw_index_labels(
        self,
        points: list[tuple[float, float]],
        labels: list[pg.TextItem],
        color: str,
        gap_indexes: set[int],
    ) -> None:
        while labels:
            self.plot.removeItem(labels.pop())
        if not self.show_index_labels.isChecked():
            return

        for index, point in enumerate(points):
            if not all(math.isfinite(value) for value in point):
                continue
            label_color = "#eb5757" if index in gap_indexes else color
            label = pg.TextItem(str(index), color=label_color, anchor=(0.0, 1.0))
            label.setPos(float(point[0]), float(point[1]))
            self.plot.addItem(label)
            labels.append(label)

    def _clear_loaded_index_labels(self) -> None:
        while self._loaded_index_labels:
            self.plot.removeItem(self._loaded_index_labels.pop())

    def _clear_recorded_index_labels(self) -> None:
        while self._recorded_index_labels:
            self.plot.removeItem(self._recorded_index_labels.pop())

    @classmethod
    def _gap_indexes(cls, points: list[tuple[float, float]], max_segment_m: float) -> list[int]:
        gaps: list[int] = []
        for index in range(1, len(points)):
            previous = points[index - 1]
            current = points[index]
            if not all(math.isfinite(value) for value in previous + current):
                gaps.append(index)
                continue
            if cls._distance(previous, current) > max_segment_m:
                gaps.append(index)
        return gaps

    @classmethod
    def _polyline_array(cls, points: list[tuple[float, float]], max_segment_m: float) -> np.ndarray:
        if not points:
            return np.empty((0, 2))

        output: list[tuple[float, float]] = []
        previous: tuple[float, float] | None = None
        for raw_x, raw_y in points:
            point = (float(raw_x), float(raw_y))
            if not all(math.isfinite(value) for value in point):
                previous = None
                if output and not math.isnan(output[-1][0]):
                    output.append((math.nan, math.nan))
                continue
            if previous is not None and cls._distance(previous, point) > max_segment_m:
                output.append((math.nan, math.nan))
            output.append(point)
            previous = point
        return np.array(output)

    def _selected_pose(self, state: RobotState) -> tuple[float, float]:
        return state.fused_x_m, state.fused_y_m
