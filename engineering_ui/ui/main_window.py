from __future__ import annotations

import math

from PySide6.QtCore import QSettings, QTimer, Qt
from PySide6.QtWidgets import (
    QComboBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSplitter,
    QStatusBar,
    QTabWidget,
    QToolBar,
    QVBoxLayout,
    QWidget,
)

from ble.map_download import MapDownload
from ble.client import BleRobotClient
from ble.protocol import (
    AUTH_TOKEN,
    CommandClass,
    ErrorCode,
    MAP_CHUNK_MAX_POINTS,
    MAP_MAX_POINTS,
    ODOMETRY_SOURCE_FUSED,
    TelemetryId,
    pack_control_aux_percent,
    pack_control_auto_track_config,
    pack_control_battery_compensation_enabled,
    pack_control_pid,
    pack_control_race_plan,
    pack_control_save_pid,
    pack_control_start_auto_track,
    pack_control_start_line,
    pack_control_start_map,
    pack_control_stop,
    pack_map_delete,
    pack_line_calibrate,
    pack_line_filter,
    pack_line_threshold,
    pack_line_track_type,
    pack_odometry_position,
    pack_odometry_source,
    pack_map_record_save,
    pack_map_record_start,
    pack_map_record_stop,
    pack_read_map_chunk,
    pack_read_map_list,
    pack_read_map_record_chunk,
    pack_rgb_led_enabled,
    pack_rgb_led_manual,
    pack_rgb_led_mode,
    pack_safety_battery_block_enabled,
    pack_safety_battery_block_percent,
    pack_safety_ble_loss_enabled,
    pack_safety_collision_enabled,
    pack_safety_distance_limit,
    pack_safety_distance_limit_enabled,
    pack_safety_reset_distance,
    pack_safety_line_loss_enabled,
    pack_safety_line_loss_timeout,
    pack_safety_roll_limit,
    pack_save_map_chunk,
    pack_zero_brake_enabled,
    pack_portal_config,
    pack_telemetry_enabled,
    unpack_battery_telemetry,
    unpack_control_telemetry,
    unpack_encoder_telemetry,
    unpack_imu_fast_telemetry,
    unpack_imu_telemetry,
    unpack_line_fast_telemetry,
    unpack_line_telemetry,
    unpack_map_chunk,
    unpack_map_list,
    unpack_map_record_chunk,
    unpack_odometry_telemetry,
    unpack_packet,
    unpack_pose_telemetry,
    unpack_rgb_led_telemetry,
    unpack_safety_telemetry,
    unpack_system_telemetry,
    unpack_portal_telemetry,
    unpack_telemetry_bundle,
)
from commands import robot_commands
from telemetry.state import RobotState
from ui.control_panel import ControlPanel
from ui.imu_calibration_dialog import ImuCalibrationDialog
from ui.line_sensor_panel import LineSensorPanel
from ui.map_view import MapView
from ui.navigation_control_panel import NavigationControlPanel
from ui.plots import PlotsPanel
from ui.race_plan_panel import RacePlanPanel
from ui.rgb_led_panel import RgbLedPanel
from ui.robot_3d_view import Robot3DView
from ui.safety_panel import SafetyPanel
from ui.portal_sensor_panel import PortalSensorPanel
from ui.telemetry_panel import TelemetryPanel
from ui.telemetry_config_panel import TelemetryConfigPanel

UI_REFRESH_TARGET_HZ = 60
UI_REFRESH_PERIOD_MS = round(1000 / UI_REFRESH_TARGET_HZ)
MOTOR_COMMAND_PERIOD_MS = 40
MAP_MAX_SEGMENT_M = 0.30


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Aspirador de Pista - UI de Engenharia")
        self.state = RobotState(mode="ble")
        self.settings = QSettings("AspiradorDePista", "EngineeringUI")
        self.state.mag_ignored = self.settings.value("imu/mag_ignored", self.state.mag_ignored, type=bool)
        self.ble = BleRobotClient(self)
        self._loading_maps: dict[int, dict] = {}
        self._map_download = MapDownload()
        self._map_download_timer = QTimer(self)
        self._map_download_timer.setSingleShot(True)
        self._map_download_timer.timeout.connect(self._request_next_map_chunk)
        self._available_maps: list[dict] = []
        self._map_list_request_pending = False
        self._map_record_next_offset = 0
        self._map_record_poll_enabled = False
        self._map_record_timer = QTimer(self)
        self._map_record_timer.setSingleShot(True)
        self._map_record_timer.timeout.connect(self._poll_map_record)
        self._pending_motor_pwm: dict[str, int] = {}
        self._command_epoch = 0
        self._pending_status_action: str | None = None

        self._build_ui()
        self._wire_events()
        self._apply_style()

        self.ui_timer = QTimer(self)
        self.ui_timer.setTimerType(Qt.PreciseTimer)
        self.ui_timer.timeout.connect(self._refresh_ui)
        self.ui_timer.start(UI_REFRESH_PERIOD_MS)

        self.motor_command_timer = QTimer(self)
        self.motor_command_timer.timeout.connect(self._flush_motor_pwm)
        self.motor_command_timer.start(MOTOR_COMMAND_PERIOD_MS)

    def _build_ui(self) -> None:
        toolbar = QToolBar("BLE")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        self.scan_button = QPushButton("Scan")
        self.connect_button = QPushButton("Conectar")
        self.disconnect_button = QPushButton("Desconectar")
        self.auth_button = QPushButton("Autenticar")
        self.device_combo = QComboBox()
        self.device_combo.setMinimumWidth(260)
        self.token_input = QLineEdit(AUTH_TOKEN)
        self.token_input.setEchoMode(QLineEdit.Password)
        self.token_input.setMinimumWidth(180)
        self.connection_label = QLabel("ble | desconectado | sem auth")

        for widget in [
            self.scan_button,
            self.device_combo,
            self.connect_button,
            self.disconnect_button,
            QLabel("Token"),
            self.token_input,
            self.auth_button,
            self.connection_label,
        ]:
            toolbar.addWidget(widget)

        central = QWidget()
        root = QHBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)

        splitter = QSplitter(Qt.Horizontal)
        self.splitter = splitter
        self.left_panel = QWidget()
        left_layout = QVBoxLayout(self.left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(8)
        side_header = QHBoxLayout()
        side_header.setContentsMargins(0, 0, 0, 0)
        self.side_toggle_button = QPushButton("<<")
        self.side_toggle_button.setFixedWidth(42)
        self.side_toggle_button.setToolTip("Minimizar painel lateral")
        side_header.addWidget(self.side_toggle_button, 0)
        side_header.addStretch(1)
        left_layout.addLayout(side_header)

        self.compact_stop_button = QPushButton("STOP")
        self.compact_stop_button.setObjectName("stopButton")
        self.compact_stop_button.setMinimumHeight(42)
        self.compact_stop_button.setToolTip("Parada de emergencia do robo")
        self.compact_stop_button.hide()
        left_layout.addWidget(self.compact_stop_button, 0)

        self.control_panel = ControlPanel()
        self.telemetry_panel = TelemetryPanel()

        self.telemetry_scroll = QScrollArea()
        self.telemetry_scroll.setWidgetResizable(True)
        self.telemetry_scroll.setFrameShape(QScrollArea.NoFrame)
        self.telemetry_scroll.setWidget(self.telemetry_panel)

        left_layout.addWidget(self.control_panel, 0)
        left_layout.addWidget(self.telemetry_scroll, 1)
        self.left_panel.setMinimumWidth(390)

        tabs = QTabWidget()
        self.tabs = tabs
        self.plots_panel = PlotsPanel()
        self.map_view = MapView()
        self.navigation_control = NavigationControlPanel()
        self.race_plan_panel = RacePlanPanel()
        self.robot_view = Robot3DView()
        self.line_sensor_panel = LineSensorPanel()
        self.rgb_led_panel = RgbLedPanel()
        self.safety_panel = SafetyPanel()
        self.portal_sensor_panel = PortalSensorPanel()
        self.telemetry_config_panel = TelemetryConfigPanel(self.settings)
        tabs.addTab(self.plots_panel, "Graficos")
        tabs.addTab(self.map_view, "Odometria")
        tabs.addTab(self.navigation_control, "Controle")
        tabs.addTab(self.race_plan_panel, "Plano de corrida")
        tabs.addTab(self.robot_view, "IMU")
        tabs.addTab(self.line_sensor_panel, "Linha")
        tabs.addTab(self.rgb_led_panel, "LED")
        tabs.addTab(self.safety_panel, "Seguranca")
        tabs.addTab(self.portal_sensor_panel, "Portal")
        tabs.addTab(self.telemetry_config_panel, "Telemetria BLE")

        splitter.addWidget(self.left_panel)
        splitter.addWidget(tabs)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        root.addWidget(splitter)
        self.setCentralWidget(central)

        self.setStatusBar(QStatusBar())
        self._side_collapsed = False

    def _wire_events(self) -> None:
        self.scan_button.clicked.connect(self.ble.scan)
        self.connect_button.clicked.connect(self._connect_selected)
        self.disconnect_button.clicked.connect(self.ble.disconnect)
        self.auth_button.clicked.connect(lambda: self.ble.authenticate(self.token_input.text()))

        self.ble.devices_changed.connect(self._update_devices)
        self.ble.connected_changed.connect(self._set_connected)
        self.ble.authenticated_changed.connect(self._set_authenticated)
        self.ble.error.connect(self._show_error)
        self.ble.status.connect(lambda message: self.statusBar().showMessage(message, 5000))
        self.ble.telemetry_packet.connect(self._on_telemetry_packet)

        self.control_panel.stop_requested.connect(self._stop_now)
        self.compact_stop_button.clicked.connect(self._stop_now)
        self.side_toggle_button.clicked.connect(lambda: self._set_side_collapsed(not self._side_collapsed))
        self.control_panel.left_pwm_changed.connect(lambda value: self._queue_motor_pwm("left", value))
        self.control_panel.right_pwm_changed.connect(lambda value: self._queue_motor_pwm("right", value))
        self.control_panel.aux_pwm_changed.connect(lambda value: self._queue_motor_pwm("aux", value))
        self.control_panel.reset_encoders_requested.connect(lambda: self._send(robot_commands.reset_encoders()))
        self.robot_view.calibrate_accel_gyro_requested.connect(lambda: self._show_imu_calibration("accel_gyro"))
        self.robot_view.calibrate_yaw_drift_requested.connect(lambda: self._show_imu_calibration("yaw_drift"))
        self.robot_view.calibrate_mag_requested.connect(lambda: self._show_imu_calibration("mag"))
        self.robot_view.calibrate_all_requested.connect(lambda: self._show_imu_calibration("all"))
        self.robot_view.mag_ignored_requested.connect(self._set_mag_ignored)
        self.line_sensor_panel.calibrate_requested.connect(lambda duration_s: self._send(pack_line_calibrate(duration_s)))
        self.line_sensor_panel.track_type_requested.connect(lambda track_type: self._send(pack_line_track_type(track_type)))
        self.line_sensor_panel.threshold_requested.connect(
            lambda threshold_percent: self._send(pack_line_threshold(threshold_percent))
        )
        self.line_sensor_panel.filter_requested.connect(
            lambda filter_percent: self._send(pack_line_filter(filter_percent))
        )
        self.rgb_led_panel.enabled_requested.connect(lambda enabled: self._send(pack_rgb_led_enabled(enabled)))
        self.rgb_led_panel.mode_requested.connect(lambda mode: self._send(pack_rgb_led_mode(mode)))
        self.rgb_led_panel.manual_requested.connect(
            lambda red, green, blue, intensity: self._send(pack_rgb_led_manual(red, green, blue, intensity))
        )
        self.safety_panel.collision_enabled_requested.connect(
            lambda enabled: self._send(pack_safety_collision_enabled(enabled))
        )
        self.safety_panel.battery_block_enabled_requested.connect(
            lambda enabled: self._send(pack_safety_battery_block_enabled(enabled))
        )
        self.safety_panel.line_loss_enabled_requested.connect(
            lambda enabled: self._send(pack_safety_line_loss_enabled(enabled))
        )
        self.safety_panel.ble_loss_enabled_requested.connect(
            lambda enabled: self._send(pack_safety_ble_loss_enabled(enabled))
        )
        self.safety_panel.roll_limit_requested.connect(lambda limit: self._send(pack_safety_roll_limit(limit)))
        self.safety_panel.battery_block_percent_requested.connect(
            lambda percent: self._send(pack_safety_battery_block_percent(percent))
        )
        self.safety_panel.line_loss_timeout_requested.connect(
            lambda timeout_s: self._send(pack_safety_line_loss_timeout(timeout_s))
        )
        self.safety_panel.distance_limit_enabled_requested.connect(
            lambda enabled: self._send(pack_safety_distance_limit_enabled(enabled))
        )
        self.safety_panel.distance_limit_requested.connect(
            lambda distance_m: self._send(pack_safety_distance_limit(distance_m))
        )
        self.safety_panel.distance_reset_requested.connect(
            lambda: self._send(pack_safety_reset_distance())
        )
        self.map_view.save_map_requested.connect(self._save_map)
        self.map_view.record_start_requested.connect(self._start_map_record)
        self.map_view.record_stop_requested.connect(self._stop_map_record)
        self.map_view.record_save_requested.connect(self._save_map_record)
        self.map_view.map_list_requested.connect(self._request_map_list)
        self.map_view.map_load_requested.connect(self._load_map)
        self.map_view.map_delete_requested.connect(self._delete_map)
        self.map_view.reset_yaw_requested.connect(lambda: self._send(robot_commands.reset_yaw()))
        self.navigation_control.refresh_maps_requested.connect(self._request_map_list)
        self.navigation_control.map_load_requested.connect(self._load_map)
        self.navigation_control.map_first_point_reset_requested.connect(self._reset_odometry_position)
        self.navigation_control.start_requested.connect(self._start_control_map)
        self.navigation_control.line_start_requested.connect(self._start_control_line)
        self.navigation_control.auto_track_start_requested.connect(self._start_control_auto_track)
        self.navigation_control.stop_requested.connect(lambda: self._send(pack_control_stop()))
        self.navigation_control.pid_requested.connect(
            lambda kp, ki, kd, limit, alpha: self._send(pack_control_pid(kp, ki, kd, limit, alpha))
        )
        self.portal_sensor_panel.config_requested.connect(
            lambda enabled, threshold, speed, delay: self._send(pack_portal_config(enabled, threshold, speed, delay))
        )
        self.telemetry_config_panel.changed.connect(
            lambda message_id, enabled: self._send(pack_telemetry_enabled(message_id, enabled))
            if self.state.authenticated else None
        )
        self.navigation_control.pid_save_requested.connect(self._save_control_pid)
        self.navigation_control.aux_requested.connect(lambda aux: self._send(pack_control_aux_percent(aux)))
        self.navigation_control.battery_compensation_requested.connect(
            lambda enabled: self._send(pack_control_battery_compensation_enabled(enabled))
        )
        self.navigation_control.zero_brake_requested.connect(lambda enabled: self._send(pack_zero_brake_enabled(enabled)))
        self.race_plan_panel.refresh_maps_requested.connect(self._request_map_list)
        self.race_plan_panel.map_load_requested.connect(self._load_map)
        self.race_plan_panel.apply_requested.connect(self._apply_race_plan)
        self.race_plan_panel.start_requested.connect(self._start_race_plan)
        self.race_plan_panel.stop_requested.connect(lambda: self._send(pack_control_stop()))

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QWidget { background: #101418; color: #d6dde6; font-size: 13px; }
            QGroupBox { border: 1px solid #2b3541; border-radius: 6px; margin-top: 12px; padding: 8px; }
            QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; color: #9fb0c3; }
            QPushButton { background: #26323d; border: 1px solid #3a4654; border-radius: 6px; padding: 8px 10px; }
            QPushButton:hover { background: #314151; }
            QPushButton:checked { background: #70422a; border-color: #b56a3f; }
            QPushButton#stopButton { background: #b4232d; border-color: #d8454f; font-weight: 700; }
            QLineEdit, QComboBox, QDoubleSpinBox { background: #111820; border: 1px solid #3a4654; border-radius: 5px; padding: 5px; }
            QScrollArea { border: 0; }
            QTabWidget::pane { border: 1px solid #2b3541; }
            QTabBar::tab { background: #18212a; padding: 8px 14px; border: 1px solid #2b3541; }
            QTabBar::tab:selected { background: #26323d; }
            """
        )

    def _connect_selected(self) -> None:
        address = self.device_combo.currentData()
        if not address:
            self._show_error("Nenhum dispositivo selecionado")
            return
        selected = next((device for device in self.ble.devices if device.address == address), None)
        if selected is not None and not selected.is_robot:
            self._show_error("Dispositivo selecionado nao parece ser o robo. Rode scan e selecione [ROBO].")
            return
        self.ble.connect(address)

    def _update_devices(self, devices: list) -> None:
        self.device_combo.clear()
        for device in devices:
            prefix = "[ROBO] " if device.is_robot else ""
            self.device_combo.addItem(f"{prefix}{device.name}  {device.address}", device.address)
        self.statusBar().showMessage(f"{len(devices)} dispositivo(s) encontrados", 3500)

    def _set_connected(self, connected: bool) -> None:
        self.state.connected = connected
        self.state.mode = "ble"
        if not connected:
            self._map_download.cancel()
            self._map_download_timer.stop()
            self._map_record_poll_enabled = False
            self._map_record_timer.stop()
            self._command_epoch += 1
            self._pending_motor_pwm.clear()
            self.navigation_control.cancel_pending_commands()
            self.state.authenticated = False
        self._refresh_status()
        if connected:
            QTimer.singleShot(300, lambda: self.ble.authenticate(self.token_input.text()))

    def _set_authenticated(self, authenticated: bool) -> None:
        self.state.authenticated = authenticated
        self._refresh_status()
        if authenticated:
            for message_id, enabled in self.telemetry_config_panel.selections():
                self._send(pack_telemetry_enabled(message_id, enabled))
            self._request_map_list(700)

    def _send(self, packet: bytes) -> None:
        if self.ble._is_stop_command(packet):
            self._map_download.cancel()
            self._map_download_timer.stop()
            self._command_epoch += 1
            self._pending_motor_pwm.clear()
            self.navigation_control.cancel_pending_commands()
        self.ble.write_command(packet)

    def _send_later(self, delay_ms: int, packet: bytes) -> None:
        epoch = self._command_epoch
        QTimer.singleShot(delay_ms, lambda: self._send(packet) if epoch == self._command_epoch else None)

    def _queue_motor_pwm(self, motor: str, value: int) -> None:
        self._pending_motor_pwm[motor] = int(value)

    def _flush_motor_pwm(self) -> None:
        if not self._pending_motor_pwm:
            return

        pending = self._pending_motor_pwm
        self._pending_motor_pwm = {}
        if "left" in pending:
            self._send(robot_commands.set_left_pwm(pending["left"]))
        if "right" in pending:
            self._send(robot_commands.set_right_pwm(pending["right"]))
        if "aux" in pending:
            self._send(robot_commands.set_aux_pwm(pending["aux"]))

    def _stop_now(self) -> None:
        self._pending_motor_pwm.clear()
        self._send(robot_commands.stop())

    def _set_side_collapsed(self, collapsed: bool) -> None:
        self._side_collapsed = collapsed
        self.control_panel.setVisible(not collapsed)
        self.telemetry_scroll.setVisible(not collapsed)
        self.compact_stop_button.setVisible(collapsed)
        self.side_toggle_button.setText(">>" if collapsed else "<<")
        self.side_toggle_button.setToolTip("Expandir painel lateral" if collapsed else "Minimizar painel lateral")
        if collapsed:
            self.left_panel.setMinimumWidth(64)
            self.left_panel.setMaximumWidth(72)
            self.splitter.setSizes([72, max(800, self.width() - 72)])
        else:
            self.left_panel.setMaximumWidth(16777215)
            self.left_panel.setMinimumWidth(390)
            self.splitter.setSizes([390, max(800, self.width() - 390)])

    def _request_map_list(self, delay_ms: int = 450) -> None:
        if self._map_list_request_pending:
            return
        self._map_list_request_pending = True

        def send_request() -> None:
            self._map_list_request_pending = False
            self._send(pack_read_map_list())

        QTimer.singleShot(delay_ms, send_request)

    def _refresh_status(self) -> None:
        auth = "auth" if self.state.authenticated else "sem auth"
        conn = "conectado" if self.state.connected else "desconectado"
        self.connection_label.setText(f"{self.state.mode} | {conn} | {auth}")

    def _refresh_ui(self) -> None:
        self.telemetry_panel.refresh(self.state)
        current_tab = self.tabs.currentWidget()
        if current_tab is self.plots_panel:
            self.plots_panel.refresh(self.state)
        elif current_tab is self.map_view:
            self.map_view.refresh(self.state)
        elif current_tab is self.navigation_control:
            self.navigation_control.refresh(self.state)
        elif current_tab is self.race_plan_panel:
            self.race_plan_panel.refresh(self.state)
        elif current_tab is self.robot_view:
            self.robot_view.refresh(self.state)
        elif current_tab is self.line_sensor_panel:
            self.line_sensor_panel.refresh(self.state)
        elif current_tab is self.rgb_led_panel:
            self.rgb_led_panel.refresh(self.state)
        elif current_tab is self.safety_panel:
            self.safety_panel.refresh(self.state)
        elif current_tab is self.portal_sensor_panel:
            self.portal_sensor_panel.refresh(self.state)
        self._refresh_status()

    def _show_error(self, message: str) -> None:
        self.state.last_error = message
        self.statusBar().showMessage(message, 5000)
        if "Nenhum dispositivo" in message:
            QMessageBox.warning(self, "BLE", message)

    def _show_imu_calibration(self, mode: str) -> None:
        durations = {
            "accel_gyro": 5,
            "yaw_drift": 20,
            "mag": 30,
            "all": 45,
        }
        dialog = ImuCalibrationDialog(mode=mode, duration_s=durations.get(mode, 45), parent=self)
        if mode == "accel_gyro":
            dialog.start_requested.connect(lambda duration_s: self._send(robot_commands.calibrate_imu_accel_gyro(duration_s)))
        elif mode == "yaw_drift":
            dialog.start_requested.connect(lambda duration_s: self._send(robot_commands.calibrate_imu_yaw_drift(duration_s)))
        elif mode == "mag":
            dialog.start_requested.connect(lambda duration_s: self._send(robot_commands.calibrate_imu_magnetometer(duration_s)))
        else:
            dialog.start_requested.connect(lambda duration_s: self._send(robot_commands.calibrate_imu_all(duration_s)))
        dialog.exec()

    def _set_mag_ignored(self, ignored: bool) -> None:
        self.state.mag_ignored = ignored
        self.settings.setValue("imu/mag_ignored", ignored)
        self._send(robot_commands.set_imu_mag_ignored(ignored))

    def _save_map(self, name: str, points: list[tuple[float, float]]) -> None:
        clean_name = name.strip() or "mapa"
        original_count = len(points)
        points = self._sanitize_map_points(points)
        removed_count = original_count - len(points)
        if len(points) > MAP_MAX_POINTS:
            points = points[:MAP_MAX_POINTS]
            self.statusBar().showMessage(f"Mapa limitado a {MAP_MAX_POINTS} pontos", 5000)
        if not points:
            self._show_error("Mapa sem pontos para salvar")
            return
        distance_m = sum(
            ((points[index][0] - points[index - 1][0]) ** 2 + (points[index][1] - points[index - 1][1]) ** 2) ** 0.5
            for index in range(1, len(points))
        )
        if distance_m <= 0.005:
            self._show_error("Mapa com distancia zero: verifique se a odometria esta variando antes de salvar")
            return

        delay_ms = 0
        for offset in range(0, len(points), MAP_CHUNK_MAX_POINTS):
            chunk = points[offset:offset + MAP_CHUNK_MAX_POINTS]
            packet = pack_save_map_chunk(clean_name, len(points), offset, chunk)
            self._send_later(delay_ms, packet)
            delay_ms += 140
        suffix = f", {removed_count} salto(s) removido(s)" if removed_count else ""
        self.statusBar().showMessage(f"Salvando mapa {clean_name} ({len(points)} pontos{suffix})", 5000)
        QTimer.singleShot(delay_ms + 900, lambda: self._request_map_list(600))

    def _start_map_record(self, name: str) -> None:
        clean_name = name.strip() or "mapa"
        self._map_record_next_offset = 0
        self._map_record_poll_enabled = True
        self._send(pack_map_record_start(clean_name))
        self._schedule_map_record_poll(300)
        self.statusBar().showMessage(f"Gravacao no ESP iniciada: {clean_name}", 2500)

    def _stop_map_record(self) -> None:
        self._map_record_poll_enabled = True
        self._send(pack_map_record_stop())
        self._schedule_map_record_poll(100)
        self.statusBar().showMessage("Aguardando confirmacao de parada da gravacao", 2500)

    def _save_map_record(self, name: str) -> None:
        clean_name = name.strip() or "mapa"
        self._map_record_poll_enabled = False
        self._send(pack_map_record_save(clean_name))
        QTimer.singleShot(300, lambda: self._request_map_record_chunk(self._map_record_next_offset))
        QTimer.singleShot(1100, lambda: self._request_map_list(300))
        self.statusBar().showMessage(f"Salvando gravacao do ESP: {clean_name}", 3000)

    def _request_map_record_chunk(self, offset: int) -> None:
        self._send(pack_read_map_record_chunk(offset))

    def _schedule_map_record_poll(self, delay_ms: int = 500) -> None:
        self._map_record_timer.start(delay_ms)

    def _poll_map_record(self) -> None:
        if not self._map_record_poll_enabled:
            return
        self._request_map_record_chunk(self._map_record_next_offset)
        self._schedule_map_record_poll(500)

    @staticmethod
    def _sanitize_map_points(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
        cleaned: list[tuple[float, float]] = []
        for point in points:
            x_m = float(point[0])
            y_m = float(point[1])
            if not (math.isfinite(x_m) and math.isfinite(y_m)):
                continue
            current = (x_m, y_m)
            if not cleaned:
                cleaned.append(current)
                continue
            dx = current[0] - cleaned[-1][0]
            dy = current[1] - cleaned[-1][1]
            distance_m = (dx * dx + dy * dy) ** 0.5
            if distance_m <= MAP_MAX_SEGMENT_M:
                cleaned.append(current)
        return cleaned

    def _load_map(self, slot: int) -> None:
        if not self._map_download.start(slot):
            return  # Navigation and race-plan panels share this same download.
        self._map_download_timer.stop()
        self._request_next_map_chunk()

    def _request_next_map_chunk(self) -> None:
        transfer = self._map_download
        if transfer.slot is None:
            return
        if transfer.attempts >= 3:
            self._show_error("Leitura de mapa sem resposta; use Atualizar para tentar novamente")
            transfer.cancel()
            return
        transfer.attempts += 1
        self._send(pack_read_map_chunk(transfer.slot, transfer.offset))
        self._map_download_timer.start(1000)

    def _delete_map(self, slot: int, name: str) -> None:
        label = name or f"slot {slot}"
        answer = QMessageBox.question(
            self,
            "Apagar pista",
            f"Apagar a pista '{label}' da memoria do robo?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        self._send(pack_map_delete(slot))
        self.map_view.clear_loaded_map(slot)
        self.statusBar().showMessage(f"Apagando pista '{label}'", 2500)
        QTimer.singleShot(350, self._request_map_list)

    def _start_control_map(self, slot: int, speed: int, source: int) -> None:
        self._send(pack_odometry_source(source))
        self._send(pack_control_start_map(slot, speed))

    def _start_control_line(self, speed: int) -> None:
        self._send(pack_control_start_line(speed))

    def _start_control_auto_track(self, slot: int, speed: int, source: int) -> None:
        self._send(pack_odometry_source(source))
        self._send(pack_control_start_auto_track(slot, speed))

    def _race_plan_packets(
        self,
        config: dict,
        segments: list[tuple[int, int, int, int, int, int, float, float, float]],
        battery_enabled: bool,
    ) -> list[bytes]:
        packets = [
            pack_control_auto_track_config(bool(config["line_loss_odometry_enabled"])),
            pack_control_battery_compensation_enabled(battery_enabled),
        ]
        packets.extend(pack_control_race_plan(segments, True))
        return packets

    def _send_spaced(self, packets: list[bytes], interval_ms: int = 0) -> int:
        # The BLE worker serializes ATT writes; no UI timers are needed.
        for packet in packets:
            self._send(packet)
        return 0

    def _apply_race_plan(
        self,
        config: dict,
        segments: list[tuple[int, int, int, int, int, int, float, float, float]],
        battery_enabled: bool,
    ) -> None:
        try:
            packets = self._race_plan_packets(config, segments, battery_enabled)
        except (KeyError, TypeError, ValueError) as exc:
            self._show_error(f"Plano de corrida invalido: {exc}")
            return
        self._send_spaced(packets)
        self.statusBar().showMessage("Plano de corrida aplicado", 2500)

    def _start_race_plan(
        self,
        slot: int,
        speed: int,
        config: dict,
        segments: list[tuple[int, int, int, int, int, int, float, float, float]],
        battery_enabled: bool,
    ) -> None:
        try:
            packets = self._race_plan_packets(config, segments, battery_enabled)
        except (KeyError, TypeError, ValueError) as exc:
            self._show_error(f"Plano de corrida invalido: {exc}")
            return
        first_pose = self.race_plan_panel.first_point_pose()
        packets.append(pack_odometry_source(ODOMETRY_SOURCE_FUSED))
        if first_pose is not None:
            x_m, y_m, _heading_rad = first_pose
            packets.append(pack_odometry_position(x_m, y_m))
        self._send_spaced(packets)
        self._send(pack_control_start_auto_track(slot, speed, True))
        if first_pose is not None:
            self.statusBar().showMessage("Iniciando plano de corrida: posicao e angulo resetados no ponto 0", 2500)
        else:
            self.statusBar().showMessage("Iniciando plano de corrida", 2500)

    def _reset_odometry_position(self, x_m: float, y_m: float) -> None:
        self._send(pack_odometry_position(x_m, y_m))
        self.statusBar().showMessage(f"Posicao e angulo resetados: {x_m:.3f}, {y_m:.3f}", 2500)

    def _save_control_pid(self, kp: float, ki: float, kd: float, limit: int, aux: int, alpha: float) -> None:
        self._pending_status_action = "pid_save"
        self._send(pack_control_save_pid(kp, ki, kd, limit, aux, alpha))

    def _on_telemetry_packet(self, data: bytes) -> None:
        try:
            command_class, message_id, payload = unpack_packet(data)
        except ValueError as exc:
            self._show_error(f"Telemetria invalida: {exc}")
            return

        if command_class != CommandClass.TELE:
            return

        if message_id == TelemetryId.BUNDLE:
            try:
                records = unpack_telemetry_bundle(payload)
            except ValueError as exc:
                self._show_error(f"Bundle de telemetria invalido: {exc}")
                return
            for bundled_message_id, bundled_payload in records:
                self._handle_telemetry_message(bundled_message_id, bundled_payload)
            return

        self._handle_telemetry_message(message_id, payload)

    def _handle_telemetry_message(self, message_id: int, payload: bytes) -> None:
        if message_id == TelemetryId.STATUS:
            status = payload[0] if payload else -1
            try:
                status_name = ErrorCode(status).name
            except ValueError:
                status_name = f"0x{status:02x}" if status >= 0 else "vazio"
            self.statusBar().showMessage(f"Status robo: {status_name}", 2500)
            if self._pending_status_action == "pid_save":
                self.navigation_control.finish_pid_save(status == ErrorCode.OK)
                self._pending_status_action = None
        elif message_id == TelemetryId.IMU:
            try:
                self.state.update(unpack_imu_telemetry(payload))
            except ValueError as exc:
                self._show_error(f"Telemetria IMU invalida: {exc}")
        elif message_id == TelemetryId.IMU_FAST:
            try:
                self.state.update(unpack_imu_fast_telemetry(payload))
            except ValueError as exc:
                self._show_error(f"Telemetria IMU rapida invalida: {exc}")
        elif message_id == TelemetryId.ENCODERS:
            try:
                self.state.update(unpack_encoder_telemetry(payload))
            except ValueError as exc:
                self._show_error(f"Telemetria encoder invalida: {exc}")
        elif message_id == TelemetryId.BATTERY:
            try:
                self.state.update(unpack_battery_telemetry(payload))
            except ValueError as exc:
                self._show_error(f"Telemetria bateria invalida: {exc}")
        elif message_id == TelemetryId.ODOMETRY:
            try:
                self.state.update(unpack_odometry_telemetry(payload))
            except ValueError as exc:
                self._show_error(f"Telemetria odometria invalida: {exc}")
        elif message_id == TelemetryId.POSE:
            try:
                self.state.update(unpack_pose_telemetry(payload))
            except ValueError as exc:
                self._show_error(f"Telemetria posicao invalida: {exc}")
        elif message_id == TelemetryId.MAP_LIST:
            try:
                self._available_maps = unpack_map_list(payload)
                self.map_view.set_available_maps(self._available_maps)
                self.navigation_control.set_available_maps(self._available_maps)
                self.race_plan_panel.set_available_maps(self._available_maps)
            except ValueError as exc:
                self._show_error(f"Lista de mapas invalida: {exc}")
        elif message_id == TelemetryId.MAP_CHUNK:
            try:
                chunk = unpack_map_chunk(payload)
            except ValueError as exc:
                self._show_error(f"Chunk de mapa invalido: {exc}")
                return

            slot = int(chunk["slot"])
            total = int(chunk["total_points"])
            offset = int(chunk["offset"])
            point_count = int(chunk["point_count"])
            points = list(chunk["points"])
            transfer = self._map_download
            if not transfer.accept(slot, offset, point_count, total):
                return  # Old/duplicate notifications do not schedule any extra requests.
            self._map_download_timer.stop()
            if offset == 0:
                self.map_view.start_loaded_map(slot, total)
                self.navigation_control.start_loaded_map(slot, total)
                self.race_plan_panel.start_loaded_map(slot, total)
            self.map_view.set_loaded_chunk(offset, points)
            self.navigation_control.set_loaded_chunk(offset, points)
            self.race_plan_panel.set_loaded_chunk(offset, points)
            if not transfer.complete:
                self._map_download_timer.start(0)
            else:
                self.map_view.finish_loaded_map(slot)
                self.navigation_control.finish_loaded_map(slot)
                self.race_plan_panel.finish_loaded_map(slot)
                transfer.cancel()
        elif message_id == TelemetryId.MAP_RECORD_CHUNK:
            try:
                chunk = unpack_map_record_chunk(payload)
            except ValueError as exc:
                self._show_error(f"Chunk de gravacao invalido: {exc}")
                return

            total = int(chunk["total_points"])
            offset = int(chunk["offset"])
            point_count = int(chunk["point_count"])
            points = list(chunk["points"])
            active = bool(chunk["active"])
            if offset == 0:
                self._map_record_next_offset = 0
            self.map_view.set_recorded_chunk(
                total,
                offset,
                points,
                active,
                int(chunk["rejected_points"]),
                float(chunk["distance_m"]),
            )
            next_offset = offset + point_count
            if next_offset >= self._map_record_next_offset:
                self._map_record_next_offset = next_offset
            if next_offset < total:
                self._map_record_poll_enabled = True
                self._schedule_map_record_poll(100)
            elif not active and not self.map_view.recording_request_pending:
                self._map_record_poll_enabled = False
                self._map_record_timer.stop()
                self.statusBar().showMessage("Gravacao no ESP parada e sincronizada", 2500)
        elif message_id == TelemetryId.CONTROL:
            try:
                self.state.update(unpack_control_telemetry(payload))
            except ValueError as exc:
                self._show_error(f"Telemetria controle invalida: {exc}")
        elif message_id == TelemetryId.PORTAL:
            try:
                self.state.update(unpack_portal_telemetry(payload))
                self.portal_sensor_panel.record_sample(self.state)
            except ValueError as exc:
                self._show_error(f"Telemetria portal invalida: {exc}")
        elif message_id == TelemetryId.LINE:
            try:
                self.state.update(unpack_line_telemetry(payload))
            except ValueError as exc:
                self._show_error(f"Telemetria linha invalida: {exc}")
        elif message_id == TelemetryId.LINE_FAST:
            try:
                self.state.update(unpack_line_fast_telemetry(payload))
            except ValueError as exc:
                self._show_error(f"Telemetria linha rapida invalida: {exc}")
        elif message_id == TelemetryId.RGB_LED:
            try:
                self.state.update(unpack_rgb_led_telemetry(payload))
            except ValueError as exc:
                self._show_error(f"Telemetria LED invalida: {exc}")
        elif message_id == TelemetryId.SAFETY:
            try:
                self.state.update(unpack_safety_telemetry(payload))
            except ValueError as exc:
                self._show_error(f"Telemetria seguranca invalida: {exc}")
        elif message_id == TelemetryId.SYSTEM:
            try:
                self.state.update(unpack_system_telemetry(payload))
                self.navigation_control.set_zero_brake_enabled(self.state.zero_brake_enabled)
            except ValueError as exc:
                self._show_error(f"Telemetria sistema invalida: {exc}")

    def closeEvent(self, event) -> None:
        self.ble.shutdown()
        super().closeEvent(event)
