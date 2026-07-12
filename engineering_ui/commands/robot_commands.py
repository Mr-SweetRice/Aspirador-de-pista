from __future__ import annotations

import struct

from ble.protocol import CommandClass, SendId, pack_packet, pack_pwm


def stop() -> bytes:
    return pack_packet(CommandClass.SEND, SendId.STOP)


def set_left_pwm(value: int) -> bytes:
    return pack_packet(CommandClass.SEND, SendId.SET_LEFT_PWM, pack_pwm(value))


def set_right_pwm(value: int) -> bytes:
    return pack_packet(CommandClass.SEND, SendId.SET_RIGHT_PWM, pack_pwm(value))


def set_aux_pwm(value: int) -> bytes:
    return pack_packet(CommandClass.SEND, SendId.SET_AUX_PWM, pack_pwm(value))


def reset_encoders() -> bytes:
    return pack_packet(CommandClass.SEND, SendId.RESET_ENCODERS)


def reset_yaw() -> bytes:
    return pack_packet(CommandClass.SEND, SendId.RESET_YAW)


def calibrate_imu_magnetometer(duration_s: int = 30) -> bytes:
    duration_ms = max(5_000, min(120_000, duration_s * 1000))
    return pack_packet(CommandClass.SEND, SendId.IMU_CALIBRATE_MAG, struct.pack("<I", duration_ms))


def calibrate_imu_accel_gyro(duration_s: int = 5) -> bytes:
    duration_ms = max(1_000, min(30_000, duration_s * 1000))
    return pack_packet(CommandClass.SEND, SendId.IMU_CALIBRATE_ACCEL_GYRO, struct.pack("<I", duration_ms))


def calibrate_imu_yaw_drift(duration_s: int = 20) -> bytes:
    duration_ms = max(1_000, min(120_000, duration_s * 1000))
    return pack_packet(CommandClass.SEND, SendId.IMU_CALIBRATE_YAW_DRIFT, struct.pack("<I", duration_ms))


def calibrate_imu_all(duration_s: int = 45) -> bytes:
    duration_ms = max(15_000, min(180_000, duration_s * 1000))
    return pack_packet(CommandClass.SEND, SendId.IMU_CALIBRATE_ALL, struct.pack("<I", duration_ms))


def set_imu_mag_filter_gain(gain: float) -> bytes:
    clamped_gain = max(0.0, min(0.2, gain))
    return pack_packet(CommandClass.SEND, SendId.IMU_SET_MAG_FILTER_GAIN, struct.pack("<f", clamped_gain))


def set_imu_mag_heading_mode(mode: int) -> bytes:
    clamped_mode = max(0, min(2, int(mode)))
    return pack_packet(CommandClass.SEND, SendId.IMU_SET_MAG_HEADING_MODE, struct.pack("<B", clamped_mode))


def set_imu_mag_ignored(ignored: bool) -> bytes:
    return pack_packet(CommandClass.SEND, SendId.IMU_SET_MAG_IGNORED, struct.pack("<B", 1 if ignored else 0))


def set_imu_yaw_drift_threshold(threshold_dps: float) -> bytes:
    clamped_threshold = max(0.0, min(5.0, threshold_dps))
    return pack_packet(CommandClass.SEND, SendId.IMU_SET_YAW_DRIFT_THRESHOLD, struct.pack("<f", clamped_threshold))
