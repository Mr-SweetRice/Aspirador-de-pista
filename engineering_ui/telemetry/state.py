from __future__ import annotations

from dataclasses import dataclass, field
from time import time

import numpy as np

HISTORY_SAMPLE_PERIOD_S = 1.0 / 120.0


@dataclass
class RobotState:
    connected: bool = False
    authenticated: bool = False
    mode: str = "ble"
    last_error: str = ""

    left_encoder: int = 0
    right_encoder: int = 0
    left_rpm: float = 0.0
    right_rpm: float = 0.0
    linear_mps: float = 0.0
    battery_v: float = 0.0
    battery_percent: float = 0.0
    battery_raw: int = 0
    cpu0_percent: float = 0.0
    cpu1_percent: float = 0.0
    zero_brake_enabled: bool = True

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    mag_ignored: bool = False

    x_m: float = 0.0
    y_m: float = 0.0
    heading_rad: float = 0.0
    angular_rad_s: float = 0.0
    encoder_x_m: float = 0.0
    encoder_y_m: float = 0.0
    encoder_heading_rad: float = 0.0
    imu_x_m: float = 0.0
    imu_y_m: float = 0.0
    imu_heading_rad: float = 0.0
    fused_x_m: float = 0.0
    fused_y_m: float = 0.0
    fused_heading_rad: float = 0.0
    odometry_imu_available: bool = False

    control_running: bool = False
    control_mode: int = 0
    control_map_slot: int = 0
    control_target_index: int = 0
    control_point_count: int = 0
    control_speed_percent: int = 0
    control_active_speed_percent: int = 0
    control_target_x_m: float = 0.0
    control_target_y_m: float = 0.0
    control_distance_m: float = 0.0
    control_angle_error_rad: float = 0.0
    control_steer_percent: float = 0.0
    control_kp: float = 35.0
    control_ki: float = 0.0
    control_kd: float = 0.0
    control_line_alpha: float = 0.7
    control_motor_limit_percent: int = 100
    control_loop_hz: float = 0.0
    race_plan_loop_hz: float = 0.0
    track_odometry_loop_hz: float = 0.0
    line_sensor_loop_hz: float = 0.0
    imu_loop_hz: float = 0.0
    control_speed_profile_enabled: bool = True
    control_aux_percent: int = 0
    control_active_aux_percent: int = 0
    control_average_speed_mps: float = 0.0
    control_max_speed_mps: float = 0.0
    control_race_plan_average_speed_mps: float = 0.0
    control_battery_compensation_enabled: bool = False
    control_map_pose_valid: bool = False
    control_map_x_m: float = 0.0
    control_map_y_m: float = 0.0
    control_map_heading_rad: float = 0.0
    control_race_segment_active: bool = False
    control_race_segment_type: int = 0
    control_race_segment_start_index: int = 0
    control_race_segment_end_index: int = 0
    control_race_segment_speed_percent: int = 0
    control_race_segment_max_speed_percent: int = 0
    control_race_segment_aux_percent: int = 0

    line_raw: tuple[int, ...] = (0, 0, 0, 0, 0, 0, 0, 0)
    line_calibrated: tuple[int, ...] = (0, 0, 0, 0, 0, 0, 0, 0)
    line_values: tuple[int, ...] = (0, 0, 0, 0, 0, 0, 0, 0)
    line_position: int = 0
    line_track_type: int = 0
    line_visible: bool = False
    line_calibrated_valid: bool = False
    line_calibrating: bool = False
    line_threshold_percent: int = 0
    line_filter_percent: int = 100
    line_read_hz: float = 0.0

    rgb_led_mode: int = 1
    rgb_led_red: int = 255
    rgb_led_green: int = 255
    rgb_led_blue: int = 255
    rgb_led_intensity: int = 32
    rgb_led_enabled: bool = True

    safety_collision_enabled: bool = True
    safety_battery_block_enabled: bool = True
    safety_line_loss_enabled: bool = True
    safety_ble_loss_enabled: bool = True
    safety_distance_limit_enabled: bool = False
    safety_collision_active: bool = False
    safety_battery_block_active: bool = False
    safety_line_loss_active: bool = False
    safety_ble_loss_active: bool = False
    safety_distance_limit_active: bool = False
    safety_motors_blocked: bool = False
    safety_line_visible: bool = False
    safety_ble_connected: bool = False
    safety_roll_limit_deg: float = 6.0
    safety_battery_block_percent: float = 10.0
    safety_line_loss_timeout_s: float = 1.0
    safety_line_loss_elapsed_s: float = 0.0
    safety_current_roll_deg: float = 0.0
    safety_current_battery_percent: float = 0.0
    safety_distance_limit_m: float = 1.0
    safety_distance_traveled_m: float = 0.0

    portal_enabled: bool = False
    portal_sensor_ok: bool = False
    portal_detected: bool = False
    portal_count: int = 0
    portal_stopping: bool = False
    portal_gpio1_available: bool = False
    portal_gpio1_active: bool = False
    portal_gpio1_events: int = 0
    portal_distance_mm: int = 0
    portal_threshold_mm: int = 300
    portal_stop_speed_percent: int = 20
    portal_stop_delay_ms: int = 500

    samples: int = 0
    time_s: list[float] = field(default_factory=list)
    rpm_history: list[tuple[float, float]] = field(default_factory=list)
    encoder_history: list[tuple[int, int]] = field(default_factory=list)
    battery_history: list[float] = field(default_factory=list)
    path_history: list[tuple[float, float]] = field(default_factory=list)
    encoder_path_history: list[tuple[float, float]] = field(default_factory=list)
    imu_path_history: list[tuple[float, float]] = field(default_factory=list)
    fused_path_history: list[tuple[float, float]] = field(default_factory=list)
    _last_history_at: float = 0.0

    def update(self, sample: dict) -> None:
        self.samples += 1
        self.portal_enabled = bool(sample.get("portal_enabled", self.portal_enabled))
        self.portal_sensor_ok = bool(sample.get("portal_sensor_ok", self.portal_sensor_ok))
        self.portal_detected = bool(sample.get("portal_detected", self.portal_detected))
        self.portal_count = int(sample.get("portal_count", self.portal_count))
        self.portal_stopping = bool(sample.get("portal_stopping", self.portal_stopping))
        if "portal_gpio1_active" in sample:
            self.portal_gpio1_available = True
        self.portal_gpio1_active = bool(sample.get("portal_gpio1_active", self.portal_gpio1_active))
        self.portal_gpio1_events = int(sample.get("portal_gpio1_events", self.portal_gpio1_events))
        self.portal_distance_mm = int(sample.get("portal_distance_mm", self.portal_distance_mm))
        self.portal_threshold_mm = int(sample.get("portal_threshold_mm", self.portal_threshold_mm))
        self.portal_stop_speed_percent = int(sample.get("portal_stop_speed_percent", self.portal_stop_speed_percent))
        self.portal_stop_delay_ms = int(sample.get("portal_stop_delay_ms", self.portal_stop_delay_ms))
        self.left_encoder = int(sample.get("left_encoder", self.left_encoder))
        self.right_encoder = int(sample.get("right_encoder", self.right_encoder))
        self.left_rpm = float(sample.get("left_rpm", self.left_rpm))
        self.right_rpm = float(sample.get("right_rpm", self.right_rpm))
        self.linear_mps = float(sample.get("linear_mps", self.linear_mps))
        self.battery_v = float(sample.get("battery_v", self.battery_v))
        self.battery_percent = float(sample.get("battery_percent", self.battery_percent))
        self.battery_raw = int(sample.get("battery_raw", self.battery_raw))
        self.cpu0_percent = float(sample.get("cpu0_percent", self.cpu0_percent))
        self.cpu1_percent = float(sample.get("cpu1_percent", self.cpu1_percent))
        self.zero_brake_enabled = bool(sample.get("zero_brake_enabled", self.zero_brake_enabled))
        self.roll = float(sample.get("roll", self.roll))
        self.pitch = float(sample.get("pitch", self.pitch))
        self.yaw = float(sample.get("yaw", self.yaw))
        self.mag_ignored = bool(sample.get("mag_ignored", self.mag_ignored))
        self.x_m = float(sample.get("x_m", self.x_m))
        self.y_m = float(sample.get("y_m", self.y_m))
        self.heading_rad = float(sample.get("heading_rad", self.heading_rad))
        self.angular_rad_s = float(sample.get("angular_rad_s", self.angular_rad_s))
        self.encoder_x_m = float(sample.get("encoder_x_m", self.encoder_x_m))
        self.encoder_y_m = float(sample.get("encoder_y_m", self.encoder_y_m))
        self.encoder_heading_rad = float(sample.get("encoder_heading_rad", self.encoder_heading_rad))
        self.imu_x_m = float(sample.get("imu_x_m", self.imu_x_m))
        self.imu_y_m = float(sample.get("imu_y_m", self.imu_y_m))
        self.imu_heading_rad = float(sample.get("imu_heading_rad", self.imu_heading_rad))
        self.fused_x_m = float(sample.get("fused_x_m", self.fused_x_m))
        self.fused_y_m = float(sample.get("fused_y_m", self.fused_y_m))
        self.fused_heading_rad = float(sample.get("fused_heading_rad", self.fused_heading_rad))
        self.odometry_imu_available = bool(sample.get("odometry_imu_available", self.odometry_imu_available))
        self.control_running = bool(sample.get("control_running", self.control_running))
        self.control_mode = int(sample.get("control_mode", self.control_mode))
        self.control_map_slot = int(sample.get("control_map_slot", self.control_map_slot))
        self.control_target_index = int(sample.get("control_target_index", self.control_target_index))
        self.control_point_count = int(sample.get("control_point_count", self.control_point_count))
        self.control_speed_percent = int(sample.get("control_speed_percent", self.control_speed_percent))
        self.control_active_speed_percent = int(
            sample.get("control_active_speed_percent", self.control_active_speed_percent)
        )
        self.control_target_x_m = float(sample.get("control_target_x_m", self.control_target_x_m))
        self.control_target_y_m = float(sample.get("control_target_y_m", self.control_target_y_m))
        self.control_distance_m = float(sample.get("control_distance_m", self.control_distance_m))
        self.control_angle_error_rad = float(sample.get("control_angle_error_rad", self.control_angle_error_rad))
        self.control_steer_percent = float(sample.get("control_steer_percent", self.control_steer_percent))
        self.control_kp = float(sample.get("control_kp", self.control_kp))
        self.control_ki = float(sample.get("control_ki", self.control_ki))
        self.control_kd = float(sample.get("control_kd", self.control_kd))
        self.control_line_alpha = float(sample.get("control_line_alpha", self.control_line_alpha))
        self.control_motor_limit_percent = int(sample.get("control_motor_limit_percent", self.control_motor_limit_percent))
        self.control_loop_hz = float(sample.get("control_loop_hz", self.control_loop_hz))
        self.race_plan_loop_hz = float(sample.get("race_plan_loop_hz", self.race_plan_loop_hz))
        self.track_odometry_loop_hz = float(sample.get("track_odometry_loop_hz", self.track_odometry_loop_hz))
        self.line_sensor_loop_hz = float(sample.get("line_sensor_loop_hz", self.line_sensor_loop_hz))
        self.imu_loop_hz = float(sample.get("imu_loop_hz", self.imu_loop_hz))
        self.control_speed_profile_enabled = bool(
            sample.get("control_speed_profile_enabled", self.control_speed_profile_enabled)
        )
        self.control_aux_percent = int(sample.get("control_aux_percent", self.control_aux_percent))
        self.control_active_aux_percent = int(
            sample.get("control_active_aux_percent", self.control_active_aux_percent)
        )
        self.control_average_speed_mps = float(
            sample.get("control_average_speed_mps", self.control_average_speed_mps)
        )
        self.control_max_speed_mps = float(sample.get("control_max_speed_mps", self.control_max_speed_mps))
        self.control_race_plan_average_speed_mps = float(
            sample.get("control_race_plan_average_speed_mps", self.control_race_plan_average_speed_mps)
        )
        self.control_battery_compensation_enabled = bool(
            sample.get("control_battery_compensation_enabled", self.control_battery_compensation_enabled)
        )
        self.control_map_pose_valid = bool(sample.get("control_map_pose_valid", self.control_map_pose_valid))
        self.control_map_x_m = float(sample.get("control_map_x_m", self.control_map_x_m))
        self.control_map_y_m = float(sample.get("control_map_y_m", self.control_map_y_m))
        self.control_map_heading_rad = float(sample.get("control_map_heading_rad", self.control_map_heading_rad))
        self.control_race_segment_active = bool(
            sample.get("control_race_segment_active", self.control_race_segment_active)
        )
        self.control_race_segment_type = int(
            sample.get("control_race_segment_type", self.control_race_segment_type)
        )
        self.control_race_segment_start_index = int(
            sample.get("control_race_segment_start_index", self.control_race_segment_start_index)
        )
        self.control_race_segment_end_index = int(
            sample.get("control_race_segment_end_index", self.control_race_segment_end_index)
        )
        self.control_race_segment_speed_percent = int(
            sample.get("control_race_segment_speed_percent", self.control_race_segment_speed_percent)
        )
        self.control_race_segment_max_speed_percent = int(
            sample.get("control_race_segment_max_speed_percent", self.control_race_segment_max_speed_percent)
        )
        self.control_race_segment_aux_percent = int(
            sample.get("control_race_segment_aux_percent", self.control_race_segment_aux_percent)
        )
        self.line_raw = tuple(sample.get("line_raw", self.line_raw))
        self.line_calibrated = tuple(sample.get("line_calibrated", self.line_calibrated))
        self.line_values = tuple(sample.get("line_values", self.line_values))
        self.line_position = int(sample.get("line_position", self.line_position))
        self.line_track_type = int(sample.get("line_track_type", self.line_track_type))
        self.line_visible = bool(sample.get("line_visible", self.line_visible))
        self.line_calibrated_valid = bool(sample.get("line_calibrated_valid", self.line_calibrated_valid))
        self.line_calibrating = bool(sample.get("line_calibrating", self.line_calibrating))
        self.line_threshold_percent = int(sample.get("line_threshold_percent", self.line_threshold_percent))
        self.line_filter_percent = int(sample.get("line_filter_percent", self.line_filter_percent))
        self.line_read_hz = float(sample.get("line_read_hz", self.line_read_hz))
        if "line_sensor_loop_hz" not in sample and "line_read_hz" in sample:
            self.line_sensor_loop_hz = self.line_read_hz
        self.rgb_led_mode = int(sample.get("rgb_led_mode", self.rgb_led_mode))
        self.rgb_led_red = int(sample.get("rgb_led_red", self.rgb_led_red))
        self.rgb_led_green = int(sample.get("rgb_led_green", self.rgb_led_green))
        self.rgb_led_blue = int(sample.get("rgb_led_blue", self.rgb_led_blue))
        self.rgb_led_intensity = int(sample.get("rgb_led_intensity", self.rgb_led_intensity))
        self.rgb_led_enabled = bool(sample.get("rgb_led_enabled", self.rgb_led_enabled))
        self.safety_collision_enabled = bool(
            sample.get("safety_collision_enabled", self.safety_collision_enabled)
        )
        self.safety_battery_block_enabled = bool(
            sample.get("safety_battery_block_enabled", self.safety_battery_block_enabled)
        )
        self.safety_line_loss_enabled = bool(sample.get("safety_line_loss_enabled", self.safety_line_loss_enabled))
        self.safety_ble_loss_enabled = bool(sample.get("safety_ble_loss_enabled", self.safety_ble_loss_enabled))
        self.safety_distance_limit_enabled = bool(
            sample.get("safety_distance_limit_enabled", self.safety_distance_limit_enabled)
        )
        self.safety_collision_active = bool(sample.get("safety_collision_active", self.safety_collision_active))
        self.safety_battery_block_active = bool(
            sample.get("safety_battery_block_active", self.safety_battery_block_active)
        )
        self.safety_line_loss_active = bool(sample.get("safety_line_loss_active", self.safety_line_loss_active))
        self.safety_ble_loss_active = bool(sample.get("safety_ble_loss_active", self.safety_ble_loss_active))
        self.safety_distance_limit_active = bool(
            sample.get("safety_distance_limit_active", self.safety_distance_limit_active)
        )
        self.safety_motors_blocked = bool(sample.get("safety_motors_blocked", self.safety_motors_blocked))
        self.safety_line_visible = bool(sample.get("safety_line_visible", self.safety_line_visible))
        self.safety_ble_connected = bool(sample.get("safety_ble_connected", self.safety_ble_connected))
        self.safety_roll_limit_deg = float(sample.get("safety_roll_limit_deg", self.safety_roll_limit_deg))
        self.safety_battery_block_percent = float(
            sample.get("safety_battery_block_percent", self.safety_battery_block_percent)
        )
        self.safety_line_loss_timeout_s = float(
            sample.get("safety_line_loss_timeout_s", self.safety_line_loss_timeout_s)
        )
        self.safety_line_loss_elapsed_s = float(
            sample.get("safety_line_loss_elapsed_s", self.safety_line_loss_elapsed_s)
        )
        self.safety_current_roll_deg = float(sample.get("safety_current_roll_deg", self.safety_current_roll_deg))
        self.safety_current_battery_percent = float(
            sample.get("safety_current_battery_percent", self.safety_current_battery_percent)
        )
        self.safety_distance_limit_m = float(
            sample.get("safety_distance_limit_m", self.safety_distance_limit_m)
        )
        self.safety_distance_traveled_m = float(
            sample.get("safety_distance_traveled_m", self.safety_distance_traveled_m)
        )

        t = time()
        if self._last_history_at == 0.0 or (t - self._last_history_at) >= HISTORY_SAMPLE_PERIOD_S:
            self._last_history_at = t
            self.time_s.append(t)
            self.rpm_history.append((self.left_rpm, self.right_rpm))
            self.encoder_history.append((self.left_encoder, self.right_encoder))
            self.battery_history.append(self.battery_percent)
            self.path_history.append((self.x_m, self.y_m))
            self.encoder_path_history.append((self.encoder_x_m, self.encoder_y_m))
            self.imu_path_history.append((self.imu_x_m, self.imu_y_m))
            self.fused_path_history.append((self.fused_x_m, self.fused_y_m))

            self.time_s = self.time_s[-600:]
            self.rpm_history = self.rpm_history[-600:]
            self.encoder_history = self.encoder_history[-600:]
            self.battery_history = self.battery_history[-600:]
            self.path_history = self.path_history[-1200:]
            self.encoder_path_history = self.encoder_path_history[-1200:]
            self.imu_path_history = self.imu_path_history[-1200:]
            self.fused_path_history = self.fused_path_history[-1200:]

    def plot_time(self) -> np.ndarray:
        if not self.time_s:
            return np.array([])
        t0 = self.time_s[0]
        return np.array(self.time_s) - t0
