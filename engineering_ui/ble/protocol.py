from __future__ import annotations

import struct
from enum import IntEnum


DEVICE_NAME = "AspiradorPista-S3"
AUTH_TOKEN = "engineering-token"
MAP_NAME_MAX_LEN = 15
MAP_MAX_POINTS = 2048
MAP_CHUNK_MAX_POINTS = 12
CONTROL_MAX_PID_GAIN = 1000.0
CONTROL_MAX_CORRECTION_PERCENT = 100.0
RACE_PLAN_MAX_SEGMENTS = 64
RACE_PLAN_CHUNK_MAX_SEGMENTS = 10

SERVICE_UUID = "5d7a0000-8f5a-4a7d-9d4f-7a6c2b8d0001"
AUTH_UUID = "5d7a0001-8f5a-4a7d-9d4f-7a6c2b8d0001"
COMMAND_UUID = "5d7a0002-8f5a-4a7d-9d4f-7a6c2b8d0001"
TELEMETRY_UUID = "5d7a0003-8f5a-4a7d-9d4f-7a6c2b8d0001"

PROTOCOL_VERSION = 1
HEADER = struct.Struct("<BBBB")


class CommandClass(IntEnum):
    SAVE = 0x01
    READ = 0x02
    SEND = 0x03
    TELE = 0x04


class ErrorCode(IntEnum):
    OK = 0x00
    AUTH_REQUIRED = 0x01
    AUTH_DENIED = 0x02
    INVALID_PACKET = 0x03
    UNSUPPORTED = 0x04
    INTERNAL = 0x05


class SaveId(IntEnum):
    MAP_CHUNK = 0x01


class ReadId(IntEnum):
    MAP_LIST = 0x01
    MAP_CHUNK = 0x02
    MAP_RECORD_CHUNK = 0x03


class SendId(IntEnum):
    STOP = 0x01
    MOVE_FORWARD = 0x02
    MOVE_BACKWARD = 0x03
    SET_LEFT_PWM = 0x10
    SET_RIGHT_PWM = 0x11
    SET_AUX_PWM = 0x12
    RESET_ENCODERS = 0x20
    RESET_YAW = 0x21
    MAP_RECORD_START = 0x22
    MAP_RECORD_STOP = 0x23
    MAP_RECORD_SAVE = 0x24
    MAP_DELETE = 0x25
    CONTROL_START_MAP = 0x40
    CONTROL_STOP = 0x41
    CONTROL_SET_PID = 0x42
    ODOMETRY_SET_SOURCE = 0x43
    CONTROL_START_LINE = 0x44
    CONTROL_SET_SPEED_PROFILE = 0x45
    CONTROL_SAVE_PID = 0x46
    CONTROL_SET_SPEED_PROFILE_ENABLED = 0x47
    CONTROL_SET_AUX_PERCENT = 0x48
    ODOMETRY_SET_POSITION = 0x49
    CONTROL_SET_BATTERY_COMPENSATION_ENABLED = 0x4A
    CONTROL_START_AUTO_TRACK = 0x4B
    CONTROL_SET_AUTO_TRACK_CONFIG = 0x4C
    CONTROL_SET_RACE_PLAN = 0x4D
    CONTROL_SET_LINE_CONTROLLER = 0x4E
    IMU_CALIBRATE_MAG = 0x30
    IMU_CALIBRATE_ALL = 0x31
    IMU_CALIBRATE_ACCEL_GYRO = 0x32
    IMU_SET_MAG_FILTER_GAIN = 0x33
    IMU_SET_MAG_HEADING_MODE = 0x34
    IMU_SET_MAG_IGNORED = 0x35
    IMU_SET_YAW_DRIFT_THRESHOLD = 0x36
    IMU_CALIBRATE_YAW_DRIFT = 0x37
    LINE_CALIBRATE = 0x50
    LINE_SET_TRACK_TYPE = 0x51
    LINE_SET_THRESHOLD = 0x52
    RGB_LED_SET_ENABLED = 0x60
    RGB_LED_SET_MODE = 0x61
    RGB_LED_SET_MANUAL = 0x62
    SAFETY_SET_COLLISION_ENABLED = 0x70
    SAFETY_SET_BATTERY_BLOCK_ENABLED = 0x71
    SAFETY_SET_ROLL_LIMIT = 0x72
    SAFETY_SET_BATTERY_BLOCK_PERCENT = 0x73
    SAFETY_SET_LINE_LOSS_ENABLED = 0x74
    SAFETY_SET_LINE_LOSS_TIMEOUT = 0x75
    SAFETY_SET_BLE_LOSS_ENABLED = 0x76


class TelemetryId(IntEnum):
    STATUS = 0x01
    ENCODERS = 0x02
    RPM = 0x03
    LINE = 0x04
    ODOMETRY = 0x05
    BATTERY = 0x06
    IMU = 0x07
    BUNDLE = 0x08
    POSE = 0x09
    LINE_FAST = 0x0A
    IMU_FAST = 0x0B
    MAP_LIST = 0x20
    MAP_CHUNK = 0x21
    MAP_RECORD_CHUNK = 0x22
    CONTROL = 0x30
    RGB_LED = 0x31
    SAFETY = 0x32
    SYSTEM = 0x33


IMU_TELEMETRY_FRAME = struct.Struct("<" + ("f" * 24) + "BB")
IMU_TELEMETRY_LEGACY_FRAME = struct.Struct("<" + ("f" * 23) + "BB")
IMU_TELEMETRY_RAW_DIAGNOSTIC_FRAME = struct.Struct("<" + ("f" * 30) + "BB")
IMU_FAST_TELEMETRY_FRAME = struct.Struct("<fffffBB")
ENCODER_TELEMETRY_FRAME = struct.Struct("<iifff")
ENCODER_TELEMETRY_LEGACY_FRAME = struct.Struct("<iiff")
BATTERY_TELEMETRY_FRAME = struct.Struct("<ffH")
BATTERY_TELEMETRY_LEGACY_FRAME = struct.Struct("<ff")
POSE_TELEMETRY_FRAME = struct.Struct("<fff")
ODOMETRY_TELEMETRY_FRAME = struct.Struct("<" + ("f" * 14) + "B")
ODOMETRY_TELEMETRY_PRE_IMU_AVAILABLE_FRAME = struct.Struct("<" + ("f" * 14))
ODOMETRY_TELEMETRY_LEGACY_FRAME = struct.Struct("<fffff")
MAP_POINT_FRAME = struct.Struct("<ff")
MAP_SAVE_CHUNK_HEADER = struct.Struct("<16sHHB")
MAP_READ_CHUNK_FRAME = struct.Struct("<BH")
MAP_LIST_ENTRY_FRAME = struct.Struct("<BHf16s")
MAP_LIST_ENTRY_LEGACY_FRAME = struct.Struct("<BH16s")
MAP_CHUNK_HEADER = struct.Struct("<BHHB")
MAP_RECORD_CHUNK_HEADER = struct.Struct("<HHBBHf")
CONTROL_START_FRAME = struct.Struct("<Bb")
CONTROL_PID_FRAME = struct.Struct("<fffB")
CONTROL_LINE_CONTROLLER_FRAME = struct.Struct("<ffffff")
CONTROL_SAVE_PID_FRAME = struct.Struct("<fffBB")
CONTROL_SAVE_LINE_CONTROLLER_FRAME = struct.Struct("<fffBBfff")
CONTROL_SPEED_PROFILE_POINT_FRAME = struct.Struct("<BfffB")
CONTROL_AUTO_TRACK_CONFIG_FRAME = struct.Struct("<BBfB")
CONTROL_RACE_PLAN_CHUNK_HEADER = struct.Struct("<BBBB")
CONTROL_RACE_PLAN_SEGMENT_FRAME = struct.Struct("<HHBBBBfff")
CONTROL_TELEMETRY_PRE_LINE_CONTROLLER_FRAME = struct.Struct("<BBHHbffffffffBfffffBBBBffBfffBBHHBBBBf")
CONTROL_TELEMETRY_FRAME = struct.Struct("<BBHHbffffffffBfffffBBBBffBfffBBHHBBBBf" + ("f" * 12))
CONTROL_TELEMETRY_PRE_TASKS_FRAME = struct.Struct("<BBHHbffffffffBfBBBBffBfffBBHHBBBBf")
CONTROL_TELEMETRY_PRE_RACE_AVERAGE_FRAME = struct.Struct("<BBHHbffffffffBfBBBBffBfffBBHHBBBB")
CONTROL_TELEMETRY_PRE_ACTIVE_SPEED_FRAME = struct.Struct("<BBHHbffffffffBfBBBBffBfffBBHHBBB")
CONTROL_TELEMETRY_PRE_RACE_SEGMENT_FRAME = struct.Struct("<BBHHbffffffffBfBBBBffBfff")
CONTROL_TELEMETRY_PRE_MAP_POSE_FRAME = struct.Struct("<BBHHbffffffffBfBBBBffB")
CONTROL_TELEMETRY_PRE_BATTERY_COMP_FRAME = struct.Struct("<BBHHbffffffffBfBBBBff")
CONTROL_TELEMETRY_PRE_SPEED_FRAME = struct.Struct("<BBHHbffffffffBfBBBB")
CONTROL_TELEMETRY_PRE_AUX_FRAME = struct.Struct("<BBHHbffffffffBfBB")
CONTROL_TELEMETRY_PRE_PROFILE_FRAME = struct.Struct("<BBHHbffffffffBfB")
CONTROL_TELEMETRY_PRE_MODE_FRAME = struct.Struct("<BBHHbffffffffBf")
CONTROL_TELEMETRY_LEGACY_FRAME = struct.Struct("<BBHHbffffffffB")
LINE_TELEMETRY_FRAME = struct.Struct("<" + ("H" * 24) + "HBBBf")
LINE_FAST_TELEMETRY_FRAME = struct.Struct("<HBBBf")
LINE_TELEMETRY_PRE_HZ_FRAME = struct.Struct("<" + ("H" * 24) + "HBBB")
LINE_TELEMETRY_LEGACY_FRAME = struct.Struct("<" + ("H" * 24) + "HBB")
RGB_LED_TELEMETRY_FRAME = struct.Struct("<BBBBBB")
SAFETY_TELEMETRY_FRAME = struct.Struct("<Hffffff")
SAFETY_TELEMETRY_U8_FLAGS_FRAME = struct.Struct("<Bffffff")
SAFETY_TELEMETRY_PRE_LINE_FRAME = struct.Struct("<Bffff")
SYSTEM_TELEMETRY_FRAME = struct.Struct("<ff")
IMU_FLAG_MAG_CALIBRATING = 1 << 0
IMU_FLAG_MAG_CALIBRATED = 1 << 1
IMU_FLAG_ACCEL_GYRO_CALIBRATING = 1 << 2
IMU_FLAG_ACCEL_GYRO_CALIBRATED = 1 << 3
IMU_FLAG_MAG_IGNORED = 1 << 4
IMU_FLAG_YAW_DRIFT_CALIBRATING = 1 << 5
IMU_FLAG_YAW_DRIFT_CALIBRATED = 1 << 6

MAG_HEADING_MODE_LABELS = {
    0: "XY",
    1: "XZ",
    2: "YZ",
}

ODOMETRY_SOURCE_FUSED = 2

ODOMETRY_SOURCE_LABELS = {
    ODOMETRY_SOURCE_FUSED: "Combinado",
}

CONTROL_MODE_ODOMETRY = 0
CONTROL_MODE_LINE = 1
CONTROL_MODE_AUTO_TRACK = 2

CONTROL_MODE_LABELS = {
    CONTROL_MODE_ODOMETRY: "Odometria",
    CONTROL_MODE_LINE: "Linha",
    CONTROL_MODE_AUTO_TRACK: "Auto pista",
}

RACE_SEGMENT_NORMAL = 0
RACE_SEGMENT_CURVE = 1
RACE_SEGMENT_STOP = 2
RACE_SEGMENT_INTERSECTION = 3

RACE_SEGMENT_LABELS = {
    RACE_SEGMENT_NORMAL: "Reta",
    RACE_SEGMENT_INTERSECTION: "Intersecao",
    RACE_SEGMENT_CURVE: "Curva",
    RACE_SEGMENT_STOP: "Parada",
}

LINE_TRACK_BLACK = 0
LINE_TRACK_WHITE = 1

LINE_TRACK_LABELS = {
    LINE_TRACK_BLACK: "Linha preta",
    LINE_TRACK_WHITE: "Linha branca",
}

RGB_LED_MODE_DISABLED = 0
RGB_LED_MODE_BATTERY = 1
RGB_LED_MODE_MANUAL = 2
RGB_LED_MODE_RACE_PLAN = 3

RGB_LED_MODE_LABELS = {
    RGB_LED_MODE_DISABLED: "Desligado",
    RGB_LED_MODE_BATTERY: "Bateria",
    RGB_LED_MODE_MANUAL: "Manual",
    RGB_LED_MODE_RACE_PLAN: "Plano de corrida",
}


def pack_packet(command_class: CommandClass, message_id: int, payload: bytes = b"") -> bytes:
    if len(payload) > 255:
        raise ValueError("payload maior que 255 bytes")
    return HEADER.pack(PROTOCOL_VERSION, int(command_class), message_id & 0xFF, len(payload)) + payload


def unpack_packet(data: bytes) -> tuple[CommandClass, int, bytes]:
    if len(data) < HEADER.size:
        raise ValueError("pacote curto")

    version, command_class, message_id, payload_len = HEADER.unpack_from(data)
    if version != PROTOCOL_VERSION:
        raise ValueError(f"versao invalida: {version}")
    if len(data) != HEADER.size + payload_len:
        raise ValueError("tamanho de payload invalido")

    return CommandClass(command_class), message_id, data[HEADER.size:]


def unpack_telemetry_bundle(payload: bytes) -> list[tuple[int, bytes]]:
    records: list[tuple[int, bytes]] = []
    offset = 0
    while offset < len(payload):
        if offset + 2 > len(payload):
            raise ValueError("bundle de telemetria truncado")
        message_id = payload[offset]
        payload_len = payload[offset + 1]
        offset += 2
        end = offset + payload_len
        if end > len(payload):
            raise ValueError("item de telemetria truncado")
        records.append((message_id, payload[offset:end]))
        offset = end
    return records


def unpack_imu_telemetry(payload: bytes) -> dict:
    if len(payload) == IMU_TELEMETRY_LEGACY_FRAME.size:
        (
            roll,
            pitch,
            yaw,
            mag_yaw,
            mag_yaw_xy,
            mag_yaw_xz,
            mag_yaw_yz,
            mag_yaw_error,
            mag_norm,
            mag_filter_gain,
            quat_w,
            quat_x,
            quat_y,
            quat_z,
            accel_x,
            accel_y,
            accel_z,
            gyro_x,
            gyro_y,
            gyro_z,
            mag_x,
            mag_y,
            mag_z,
            mag_heading_mode,
            flags,
        ) = IMU_TELEMETRY_LEGACY_FRAME.unpack(payload)
        yaw_drift_threshold = 0.05
    elif len(payload) == IMU_TELEMETRY_FRAME.size:
        (
            roll,
            pitch,
            yaw,
            mag_yaw,
            mag_yaw_xy,
            mag_yaw_xz,
            mag_yaw_yz,
            mag_yaw_error,
            mag_norm,
            mag_filter_gain,
            yaw_drift_threshold,
            quat_w,
            quat_x,
            quat_y,
            quat_z,
            accel_x,
            accel_y,
            accel_z,
            gyro_x,
            gyro_y,
            gyro_z,
            mag_x,
            mag_y,
            mag_z,
            mag_heading_mode,
            flags,
        ) = IMU_TELEMETRY_FRAME.unpack(payload)
    elif len(payload) == IMU_TELEMETRY_RAW_DIAGNOSTIC_FRAME.size:
        (
            roll,
            pitch,
            yaw,
            mag_yaw,
            mag_yaw_xy,
            mag_yaw_xz,
            mag_yaw_yz,
            _mag_raw_yaw_xy,
            _mag_raw_yaw_xz,
            _mag_raw_yaw_yz,
            mag_yaw_error,
            mag_norm,
            _mag_raw_norm,
            mag_filter_gain,
            quat_w,
            quat_x,
            quat_y,
            quat_z,
            accel_x,
            accel_y,
            accel_z,
            gyro_x,
            gyro_y,
            gyro_z,
            mag_x,
            mag_y,
            mag_z,
            _mag_raw_x,
            _mag_raw_y,
            _mag_raw_z,
            mag_heading_mode,
            flags,
        ) = IMU_TELEMETRY_RAW_DIAGNOSTIC_FRAME.unpack(payload)
        yaw_drift_threshold = 0.05
    else:
        raise ValueError("payload IMU com tamanho invalido")

    return {
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
        "mag_yaw": mag_yaw,
        "mag_yaw_xy": mag_yaw_xy,
        "mag_yaw_xz": mag_yaw_xz,
        "mag_yaw_yz": mag_yaw_yz,
        "mag_yaw_error": mag_yaw_error,
        "mag_norm": mag_norm,
        "mag_filter_gain": mag_filter_gain,
        "yaw_drift_threshold": yaw_drift_threshold,
        "mag_heading_mode": mag_heading_mode,
        "quaternion": (quat_w, quat_x, quat_y, quat_z),
        "accel": (accel_x, accel_y, accel_z),
        "gyro": (gyro_x, gyro_y, gyro_z),
        "mag": (mag_x, mag_y, mag_z),
        "mag_ignored": bool(flags & IMU_FLAG_MAG_IGNORED),
        "mag_calibrating": bool(flags & IMU_FLAG_MAG_CALIBRATING),
        "mag_calibrated": bool(flags & IMU_FLAG_MAG_CALIBRATED),
        "accel_gyro_calibrating": bool(flags & IMU_FLAG_ACCEL_GYRO_CALIBRATING),
        "accel_gyro_calibrated": bool(flags & IMU_FLAG_ACCEL_GYRO_CALIBRATED),
        "yaw_drift_calibrating": bool(flags & IMU_FLAG_YAW_DRIFT_CALIBRATING),
        "yaw_drift_calibrated": bool(flags & IMU_FLAG_YAW_DRIFT_CALIBRATED),
    }


def unpack_imu_fast_telemetry(payload: bytes) -> dict:
    if len(payload) != IMU_FAST_TELEMETRY_FRAME.size:
        raise ValueError("payload IMU rapida com tamanho invalido")

    roll, pitch, yaw, gyro_z, sample_hz, mag_heading_mode, flags = IMU_FAST_TELEMETRY_FRAME.unpack(payload)
    return {
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
        "gyro_z_dps": gyro_z,
        "imu_loop_hz": sample_hz,
        "mag_heading_mode": mag_heading_mode,
        "mag_ignored": bool(flags & IMU_FLAG_MAG_IGNORED),
        "mag_calibrating": bool(flags & IMU_FLAG_MAG_CALIBRATING),
        "mag_calibrated": bool(flags & IMU_FLAG_MAG_CALIBRATED),
        "accel_gyro_calibrating": bool(flags & IMU_FLAG_ACCEL_GYRO_CALIBRATING),
        "accel_gyro_calibrated": bool(flags & IMU_FLAG_ACCEL_GYRO_CALIBRATED),
        "yaw_drift_calibrating": bool(flags & IMU_FLAG_YAW_DRIFT_CALIBRATING),
        "yaw_drift_calibrated": bool(flags & IMU_FLAG_YAW_DRIFT_CALIBRATED),
    }


def unpack_encoder_telemetry(payload: bytes) -> dict:
    if len(payload) == ENCODER_TELEMETRY_FRAME.size:
        left_encoder, right_encoder, left_rpm, right_rpm, linear_mps = ENCODER_TELEMETRY_FRAME.unpack(payload)
    elif len(payload) == ENCODER_TELEMETRY_LEGACY_FRAME.size:
        left_encoder, right_encoder, left_rpm, right_rpm = ENCODER_TELEMETRY_LEGACY_FRAME.unpack(payload)
        linear_mps = 0.0
    else:
        raise ValueError("payload encoder com tamanho invalido")

    return {
        "left_encoder": left_encoder,
        "right_encoder": right_encoder,
        "left_rpm": left_rpm,
        "right_rpm": right_rpm,
        "linear_mps": linear_mps,
    }


def unpack_battery_telemetry(payload: bytes) -> dict:
    if len(payload) == BATTERY_TELEMETRY_FRAME.size:
        battery_v, battery_percent, battery_raw = BATTERY_TELEMETRY_FRAME.unpack(payload)
    elif len(payload) == BATTERY_TELEMETRY_LEGACY_FRAME.size:
        battery_v, battery_percent = BATTERY_TELEMETRY_LEGACY_FRAME.unpack(payload)
        battery_raw = int(battery_v)
    else:
        raise ValueError("payload bateria com tamanho invalido")

    return {
        "battery_v": battery_v,
        "battery_percent": battery_percent,
        "battery_raw": battery_raw,
    }


def unpack_odometry_telemetry(payload: bytes) -> dict:
    if len(payload) == ODOMETRY_TELEMETRY_FRAME.size:
        (
            x_m,
            y_m,
            heading_rad,
            linear_mps,
            angular_rad_s,
            encoder_x_m,
            encoder_y_m,
            encoder_heading_rad,
            imu_x_m,
            imu_y_m,
            imu_heading_rad,
            fused_x_m,
            fused_y_m,
            fused_heading_rad,
            imu_available,
        ) = ODOMETRY_TELEMETRY_FRAME.unpack(payload)
    elif len(payload) == ODOMETRY_TELEMETRY_PRE_IMU_AVAILABLE_FRAME.size:
        (
            x_m,
            y_m,
            heading_rad,
            linear_mps,
            angular_rad_s,
            encoder_x_m,
            encoder_y_m,
            encoder_heading_rad,
            imu_x_m,
            imu_y_m,
            imu_heading_rad,
            fused_x_m,
            fused_y_m,
            fused_heading_rad,
        ) = ODOMETRY_TELEMETRY_PRE_IMU_AVAILABLE_FRAME.unpack(payload)
        imu_available = True
    elif len(payload) == ODOMETRY_TELEMETRY_LEGACY_FRAME.size:
        x_m, y_m, heading_rad, linear_mps, angular_rad_s = ODOMETRY_TELEMETRY_LEGACY_FRAME.unpack(payload)
        encoder_x_m = x_m
        encoder_y_m = y_m
        encoder_heading_rad = heading_rad
        imu_x_m = x_m
        imu_y_m = y_m
        imu_heading_rad = heading_rad
        fused_x_m = x_m
        fused_y_m = y_m
        fused_heading_rad = heading_rad
        imu_available = True
    else:
        raise ValueError("payload odometria com tamanho invalido")

    return {
        "x_m": x_m,
        "y_m": y_m,
        "heading_rad": heading_rad,
        "linear_mps": linear_mps,
        "angular_rad_s": angular_rad_s,
        "encoder_x_m": encoder_x_m,
        "encoder_y_m": encoder_y_m,
        "encoder_heading_rad": encoder_heading_rad,
        "imu_x_m": imu_x_m,
        "imu_y_m": imu_y_m,
        "imu_heading_rad": imu_heading_rad,
        "fused_x_m": fused_x_m,
        "fused_y_m": fused_y_m,
        "fused_heading_rad": fused_heading_rad,
        "odometry_imu_available": bool(imu_available),
    }


def unpack_pose_telemetry(payload: bytes) -> dict:
    if len(payload) != POSE_TELEMETRY_FRAME.size:
        raise ValueError("payload posicao com tamanho invalido")
    fused_x_m, fused_y_m, fused_heading_rad = POSE_TELEMETRY_FRAME.unpack(payload)
    return {
        "x_m": fused_x_m,
        "y_m": fused_y_m,
        "heading_rad": fused_heading_rad,
        "fused_x_m": fused_x_m,
        "fused_y_m": fused_y_m,
        "fused_heading_rad": fused_heading_rad,
    }


def _encode_map_name(name: str) -> bytes:
    encoded = name.strip().encode("utf-8")[:MAP_NAME_MAX_LEN]
    return encoded + (b"\0" * (MAP_NAME_MAX_LEN + 1 - len(encoded)))


def _decode_map_name(raw: bytes) -> str:
    return raw.split(b"\0", 1)[0].decode("utf-8", errors="replace")


def pack_save_map_chunk(name: str, total_points: int, offset: int, points: list[tuple[float, float]]) -> bytes:
    clipped_points = points[:MAP_CHUNK_MAX_POINTS]
    payload = MAP_SAVE_CHUNK_HEADER.pack(_encode_map_name(name), total_points, offset, len(clipped_points))
    payload += b"".join(MAP_POINT_FRAME.pack(float(x), float(y)) for x, y in clipped_points)
    return pack_packet(CommandClass.SAVE, SaveId.MAP_CHUNK, payload)


def pack_read_map_list() -> bytes:
    return pack_packet(CommandClass.READ, ReadId.MAP_LIST)


def pack_read_map_chunk(slot: int, offset: int = 0) -> bytes:
    return pack_packet(CommandClass.READ, ReadId.MAP_CHUNK, MAP_READ_CHUNK_FRAME.pack(slot & 0xFF, offset))


def pack_read_map_record_chunk(offset: int = 0) -> bytes:
    return pack_packet(CommandClass.READ, ReadId.MAP_RECORD_CHUNK, struct.pack("<H", max(0, int(offset))))


def unpack_map_list(payload: bytes) -> list[dict]:
    if not payload:
        raise ValueError("payload lista mapas vazio")
    count = payload[0]
    expected = 1 + (count * MAP_LIST_ENTRY_FRAME.size)
    legacy_expected = 1 + (count * MAP_LIST_ENTRY_LEGACY_FRAME.size)
    if len(payload) == expected:
        entry_frame = MAP_LIST_ENTRY_FRAME
        has_distance = True
    elif len(payload) == legacy_expected:
        entry_frame = MAP_LIST_ENTRY_LEGACY_FRAME
        has_distance = False
    else:
        raise ValueError("payload lista mapas com tamanho invalido")

    maps = []
    offset = 1
    for _ in range(count):
        if has_distance:
            slot, point_count, distance_m, raw_name = entry_frame.unpack_from(payload, offset)
        else:
            slot, point_count, raw_name = entry_frame.unpack_from(payload, offset)
            distance_m = 0.0
        offset += entry_frame.size
        maps.append({
            "slot": slot,
            "point_count": point_count,
            "distance_m": distance_m,
            "name": _decode_map_name(raw_name),
        })
    return maps


def unpack_map_chunk(payload: bytes) -> dict:
    if len(payload) < MAP_CHUNK_HEADER.size:
        raise ValueError("payload chunk mapa curto")

    slot, total_points, offset, point_count = MAP_CHUNK_HEADER.unpack_from(payload)
    expected = MAP_CHUNK_HEADER.size + (point_count * MAP_POINT_FRAME.size)
    if len(payload) != expected:
        raise ValueError("payload chunk mapa com tamanho invalido")

    points = []
    point_offset = MAP_CHUNK_HEADER.size
    for _ in range(point_count):
        points.append(MAP_POINT_FRAME.unpack_from(payload, point_offset))
        point_offset += MAP_POINT_FRAME.size

    return {
        "slot": slot,
        "total_points": total_points,
        "offset": offset,
        "point_count": point_count,
        "points": points,
    }


def unpack_map_record_chunk(payload: bytes) -> dict:
    if len(payload) < MAP_RECORD_CHUNK_HEADER.size:
        raise ValueError("payload chunk gravacao mapa curto")

    total_points, offset, point_count, active, rejected_points, distance_m = MAP_RECORD_CHUNK_HEADER.unpack_from(payload)
    expected = MAP_RECORD_CHUNK_HEADER.size + (point_count * MAP_POINT_FRAME.size)
    if len(payload) != expected:
        raise ValueError("payload chunk gravacao mapa com tamanho invalido")

    points = []
    point_offset = MAP_RECORD_CHUNK_HEADER.size
    for _ in range(point_count):
        points.append(MAP_POINT_FRAME.unpack_from(payload, point_offset))
        point_offset += MAP_POINT_FRAME.size

    return {
        "total_points": total_points,
        "offset": offset,
        "point_count": point_count,
        "active": bool(active),
        "rejected_points": rejected_points,
        "distance_m": distance_m,
        "points": points,
    }


def pack_map_record_start(name: str) -> bytes:
    return pack_packet(CommandClass.SEND, SendId.MAP_RECORD_START, _encode_map_name(name))


def pack_map_record_stop() -> bytes:
    return pack_packet(CommandClass.SEND, SendId.MAP_RECORD_STOP)


def pack_map_record_save(name: str) -> bytes:
    return pack_packet(CommandClass.SEND, SendId.MAP_RECORD_SAVE, _encode_map_name(name))


def pack_map_delete(slot: int) -> bytes:
    return pack_packet(CommandClass.SEND, SendId.MAP_DELETE, struct.pack("<B", slot & 0xFF))


def pack_control_start_map(slot: int, speed_percent: int) -> bytes:
    speed = max(-100, min(100, int(speed_percent)))
    return pack_packet(CommandClass.SEND, SendId.CONTROL_START_MAP, CONTROL_START_FRAME.pack(slot & 0xFF, speed))


def pack_control_start_auto_track(slot: int, speed_percent: int) -> bytes:
    speed = max(-100, min(100, int(speed_percent)))
    return pack_packet(CommandClass.SEND, SendId.CONTROL_START_AUTO_TRACK, CONTROL_START_FRAME.pack(slot & 0xFF, speed))


def pack_control_start_line(speed_percent: int) -> bytes:
    speed = max(-100, min(100, int(speed_percent)))
    return pack_packet(CommandClass.SEND, SendId.CONTROL_START_LINE, struct.pack("<b", speed))


def pack_control_stop() -> bytes:
    return pack_packet(CommandClass.SEND, SendId.CONTROL_STOP)


def pack_control_pid(kp: float, ki: float, kd: float, motor_limit_percent: int) -> bytes:
    limit = max(0, min(100, int(motor_limit_percent)))
    return pack_packet(
        CommandClass.SEND,
        SendId.CONTROL_SET_PID,
        CONTROL_PID_FRAME.pack(
            max(0.0, min(CONTROL_MAX_PID_GAIN, float(kp))),
            max(0.0, min(CONTROL_MAX_PID_GAIN, float(ki))),
            max(0.0, min(CONTROL_MAX_PID_GAIN, float(kd))),
            limit,
        ),
    )


def pack_control_line_controller(
    kp: float,
    kn: float,
    kd: float,
    max_correction_percent: float,
    base_speed_percent: float,
    derivative_filter_alpha: float,
) -> bytes:
    return pack_packet(
        CommandClass.SEND,
        SendId.CONTROL_SET_LINE_CONTROLLER,
        CONTROL_LINE_CONTROLLER_FRAME.pack(
            max(0.0, min(CONTROL_MAX_PID_GAIN, float(kp))),
            max(0.0, min(CONTROL_MAX_PID_GAIN, float(kn))),
            max(0.0, min(CONTROL_MAX_PID_GAIN, float(kd))),
            max(0.0, min(CONTROL_MAX_CORRECTION_PERCENT, float(max_correction_percent))),
            max(0.0, min(100.0, abs(float(base_speed_percent)))),
            max(0.0, min(1.0, float(derivative_filter_alpha))),
        ),
    )


def pack_control_save_pid(
    kp: float,
    ki: float,
    kd: float,
    motor_limit_percent: int,
    aux_percent: int | None = None,
    line_max_correction_percent: float | None = None,
    line_base_speed_percent: float | None = None,
    line_derivative_filter_alpha: float | None = None,
) -> bytes:
    limit = max(0, min(100, int(motor_limit_percent)))
    safe_kp = max(0.0, min(CONTROL_MAX_PID_GAIN, float(kp)))
    safe_ki = max(0.0, min(CONTROL_MAX_PID_GAIN, float(ki)))
    safe_kd = max(0.0, min(CONTROL_MAX_PID_GAIN, float(kd)))
    payload = CONTROL_PID_FRAME.pack(safe_kp, safe_ki, safe_kd, limit)
    if aux_percent is not None:
        aux = max(0, min(100, int(aux_percent)))
        payload = CONTROL_SAVE_PID_FRAME.pack(safe_kp, safe_ki, safe_kd, limit, aux)
        if (
            line_max_correction_percent is not None
            and line_base_speed_percent is not None
            and line_derivative_filter_alpha is not None
        ):
            payload = CONTROL_SAVE_LINE_CONTROLLER_FRAME.pack(
                safe_kp,
                safe_ki,
                safe_kd,
                limit,
                aux,
                max(0.0, min(CONTROL_MAX_CORRECTION_PERCENT, float(line_max_correction_percent))),
                max(0.0, min(100.0, abs(float(line_base_speed_percent)))),
                max(0.0, min(1.0, float(line_derivative_filter_alpha))),
            )
    return pack_packet(
        CommandClass.SEND,
        SendId.CONTROL_SAVE_PID,
        payload,
    )


def pack_control_speed_profile(points: list[tuple[int, float, float, float, int]]) -> bytes:
    if len(points) != 5:
        raise ValueError("curva de velocidade deve ter 5 pontos")
    payload = b""
    previous_speed = -1
    for speed, kp, ki, kd, aux in points:
        speed_value = max(0, min(100, int(speed)))
        if speed_value <= previous_speed:
            raise ValueError("velocidades da curva devem ser crescentes")
        previous_speed = speed_value
        payload += CONTROL_SPEED_PROFILE_POINT_FRAME.pack(
            speed_value,
            max(0.0, min(CONTROL_MAX_PID_GAIN, float(kp))),
            max(0.0, min(CONTROL_MAX_PID_GAIN, float(ki))),
            max(0.0, min(CONTROL_MAX_PID_GAIN, float(kd))),
            max(0, min(100, int(aux))),
        )
    return pack_packet(CommandClass.SEND, SendId.CONTROL_SET_SPEED_PROFILE, payload)


def pack_control_speed_profile_enabled(enabled: bool) -> bytes:
    return pack_packet(CommandClass.SEND, SendId.CONTROL_SET_SPEED_PROFILE_ENABLED, struct.pack("<B", 1 if enabled else 0))


def pack_control_aux_percent(aux_percent: int) -> bytes:
    aux = max(0, min(100, int(aux_percent)))
    return pack_packet(CommandClass.SEND, SendId.CONTROL_SET_AUX_PERCENT, struct.pack("<B", aux))


def pack_control_battery_compensation_enabled(enabled: bool) -> bytes:
    return pack_packet(
        CommandClass.SEND,
        SendId.CONTROL_SET_BATTERY_COMPENSATION_ENABLED,
        struct.pack("<B", 1 if enabled else 0),
    )


def pack_control_auto_track_config(line_loss_odometry_enabled: bool) -> bytes:
    return pack_packet(
        CommandClass.SEND,
        SendId.CONTROL_SET_AUTO_TRACK_CONFIG,
        CONTROL_AUTO_TRACK_CONFIG_FRAME.pack(
            0,
            0,
            1.5707963,
            1 if line_loss_odometry_enabled else 0,
        ),
    )


def pack_control_race_plan(segments: list[tuple], enabled: bool) -> list[bytes]:
    if not enabled:
        payload = CONTROL_RACE_PLAN_CHUNK_HEADER.pack(0, 0, 0, 0)
        return [pack_packet(CommandClass.SEND, SendId.CONTROL_SET_RACE_PLAN, payload)]
    if not segments:
        raise ValueError("plano de corrida sem segmentos")
    if len(segments) > RACE_PLAN_MAX_SEGMENTS:
        raise ValueError(f"plano limitado a {RACE_PLAN_MAX_SEGMENTS} segmentos")

    sanitized: list[tuple[int, int, int, int, int, int, float, float, float]] = []
    previous_end = -1
    for segment in segments:
        if len(segment) == 8:
            start, end, segment_type, speed, aux, kp, ki, kd = segment
            max_speed = 0 if int(segment_type) == RACE_SEGMENT_STOP else 100
        elif len(segment) == 9:
            start, end, segment_type, speed, max_speed, aux, kp, ki, kd = segment
        else:
            raise ValueError("segmento do plano com tamanho invalido")
        start_index = max(0, min(MAP_MAX_POINTS - 1, int(start)))
        end_index = max(0, min(MAP_MAX_POINTS - 1, int(end)))
        if start_index > end_index:
            raise ValueError("segmento com inicio maior que fim")
        if start_index <= previous_end:
            raise ValueError("segmentos devem estar em ordem e sem sobreposicao")
        if int(segment_type) not in RACE_SEGMENT_LABELS:
            raise ValueError("tipo de segmento invalido")
        sanitized.append(
            (
                start_index,
                end_index,
                int(segment_type),
                max(0, min(100, int(speed))),
                max(0, min(100, int(max_speed))),
                max(0, min(100, int(aux))),
                max(0.0, min(CONTROL_MAX_PID_GAIN, float(kp))),
                max(0.0, min(CONTROL_MAX_PID_GAIN, float(ki))),
                max(0.0, min(CONTROL_MAX_PID_GAIN, float(kd))),
            )
        )
        previous_end = end_index

    packets: list[bytes] = []
    total = len(sanitized)
    for offset in range(0, total, RACE_PLAN_CHUNK_MAX_SEGMENTS):
        chunk = sanitized[offset:offset + RACE_PLAN_CHUNK_MAX_SEGMENTS]
        payload = CONTROL_RACE_PLAN_CHUNK_HEADER.pack(1, total, offset, len(chunk))
        for segment in chunk:
            payload += CONTROL_RACE_PLAN_SEGMENT_FRAME.pack(*segment)
        packets.append(pack_packet(CommandClass.SEND, SendId.CONTROL_SET_RACE_PLAN, payload))
    return packets


def pack_odometry_source(source: int) -> bytes:
    del source
    return pack_packet(CommandClass.SEND, SendId.ODOMETRY_SET_SOURCE, struct.pack("<B", ODOMETRY_SOURCE_FUSED))


def pack_odometry_position(x_m: float, y_m: float) -> bytes:
    return pack_packet(CommandClass.SEND, SendId.ODOMETRY_SET_POSITION, struct.pack("<ff", float(x_m), float(y_m)))


def pack_line_calibrate(duration_s: int = 5) -> bytes:
    duration_ms = max(1_000, min(120_000, int(duration_s) * 1000))
    return pack_packet(CommandClass.SEND, SendId.LINE_CALIBRATE, struct.pack("<I", duration_ms))


def pack_line_track_type(track_type: int) -> bytes:
    value = LINE_TRACK_WHITE if int(track_type) == LINE_TRACK_WHITE else LINE_TRACK_BLACK
    return pack_packet(CommandClass.SEND, SendId.LINE_SET_TRACK_TYPE, struct.pack("<B", value))


def pack_line_threshold(threshold_percent: int) -> bytes:
    value = max(0, min(45, int(threshold_percent)))
    return pack_packet(CommandClass.SEND, SendId.LINE_SET_THRESHOLD, struct.pack("<B", value))


def pack_rgb_led_enabled(enabled: bool) -> bytes:
    return pack_packet(CommandClass.SEND, SendId.RGB_LED_SET_ENABLED, struct.pack("<B", 1 if enabled else 0))


def pack_rgb_led_mode(mode: int) -> bytes:
    value = int(mode)
    if value not in RGB_LED_MODE_LABELS:
        value = RGB_LED_MODE_BATTERY
    return pack_packet(CommandClass.SEND, SendId.RGB_LED_SET_MODE, struct.pack("<B", value))


def pack_rgb_led_manual(red: int, green: int, blue: int, intensity: int) -> bytes:
    payload = struct.pack(
        "<BBBB",
        max(0, min(255, int(red))),
        max(0, min(255, int(green))),
        max(0, min(255, int(blue))),
        max(0, min(255, int(intensity))),
    )
    return pack_packet(CommandClass.SEND, SendId.RGB_LED_SET_MANUAL, payload)


def pack_safety_collision_enabled(enabled: bool) -> bytes:
    return pack_packet(CommandClass.SEND, SendId.SAFETY_SET_COLLISION_ENABLED, struct.pack("<B", 1 if enabled else 0))


def pack_safety_battery_block_enabled(enabled: bool) -> bytes:
    return pack_packet(CommandClass.SEND, SendId.SAFETY_SET_BATTERY_BLOCK_ENABLED, struct.pack("<B", 1 if enabled else 0))


def pack_safety_line_loss_enabled(enabled: bool) -> bytes:
    return pack_packet(CommandClass.SEND, SendId.SAFETY_SET_LINE_LOSS_ENABLED, struct.pack("<B", 1 if enabled else 0))


def pack_safety_ble_loss_enabled(enabled: bool) -> bytes:
    return pack_packet(CommandClass.SEND, SendId.SAFETY_SET_BLE_LOSS_ENABLED, struct.pack("<B", 1 if enabled else 0))


def pack_safety_roll_limit(limit_deg: float) -> bytes:
    value = max(1.0, min(90.0, float(limit_deg)))
    return pack_packet(CommandClass.SEND, SendId.SAFETY_SET_ROLL_LIMIT, struct.pack("<f", value))


def pack_safety_battery_block_percent(percent: float) -> bytes:
    value = max(0.0, min(100.0, float(percent)))
    return pack_packet(CommandClass.SEND, SendId.SAFETY_SET_BATTERY_BLOCK_PERCENT, struct.pack("<f", value))


def pack_safety_line_loss_timeout(timeout_s: float) -> bytes:
    value = max(0.1, min(10.0, float(timeout_s)))
    return pack_packet(CommandClass.SEND, SendId.SAFETY_SET_LINE_LOSS_TIMEOUT, struct.pack("<f", value))


def unpack_control_telemetry(payload: bytes) -> dict:
    control_map_pose_valid = False
    control_map_x = 0.0
    control_map_y = 0.0
    control_map_heading = 0.0
    race_segment_active = False
    race_segment_type = RACE_SEGMENT_NORMAL
    race_segment_start = 0
    race_segment_end = 0
    race_segment_speed = 0
    race_segment_max_speed = 0
    race_segment_aux = 0
    active_speed_percent = 0
    active_speed_received = False
    race_plan_average_speed_mps = 0.0
    race_plan_loop_hz = 0.0
    track_odometry_loop_hz = 0.0
    line_sensor_loop_hz = 0.0
    imu_loop_hz = 0.0
    line_error_raw = 0.0
    line_error_normalized = 0.0
    line_proportional_term = 0.0
    line_nonlinear_term = 0.0
    line_derivative_raw = 0.0
    line_derivative_filtered = 0.0
    line_correction = 0.0
    line_left_command = 0.0
    line_right_command = 0.0
    line_dt_s = 0.0
    line_max_correction = 0.0
    line_derivative_filter_alpha = 0.0
    if len(payload) == CONTROL_TELEMETRY_FRAME.size:
        (
            running,
            map_slot,
            target_index,
            point_count,
            speed,
            target_x,
            target_y,
            distance,
            error,
            steer,
            kp,
            ki,
            kd,
            motor_limit,
            loop_hz,
            race_plan_loop_hz,
            track_odometry_loop_hz,
            line_sensor_loop_hz,
            imu_loop_hz,
            mode,
            speed_profile_enabled,
            aux_percent,
            active_aux_percent,
            average_speed_mps,
            max_speed_mps,
            battery_compensation_enabled,
            control_map_x,
            control_map_y,
            control_map_heading,
            race_segment_active,
            race_segment_type,
            race_segment_start,
            race_segment_end,
            race_segment_speed,
            race_segment_max_speed,
            race_segment_aux,
            active_speed_percent,
            race_plan_average_speed_mps,
            line_error_raw,
            line_error_normalized,
            line_proportional_term,
            line_nonlinear_term,
            line_derivative_raw,
            line_derivative_filtered,
            line_correction,
            line_left_command,
            line_right_command,
            line_dt_s,
            line_max_correction,
            line_derivative_filter_alpha,
        ) = CONTROL_TELEMETRY_FRAME.unpack(payload)
        control_map_pose_valid = True
        active_speed_received = True
    elif len(payload) == CONTROL_TELEMETRY_PRE_LINE_CONTROLLER_FRAME.size:
        (
            running,
            map_slot,
            target_index,
            point_count,
            speed,
            target_x,
            target_y,
            distance,
            error,
            steer,
            kp,
            ki,
            kd,
            motor_limit,
            loop_hz,
            race_plan_loop_hz,
            track_odometry_loop_hz,
            line_sensor_loop_hz,
            imu_loop_hz,
            mode,
            speed_profile_enabled,
            aux_percent,
            active_aux_percent,
            average_speed_mps,
            max_speed_mps,
            battery_compensation_enabled,
            control_map_x,
            control_map_y,
            control_map_heading,
            race_segment_active,
            race_segment_type,
            race_segment_start,
            race_segment_end,
            race_segment_speed,
            race_segment_max_speed,
            race_segment_aux,
            active_speed_percent,
            race_plan_average_speed_mps,
        ) = CONTROL_TELEMETRY_PRE_LINE_CONTROLLER_FRAME.unpack(payload)
        control_map_pose_valid = True
        active_speed_received = True
    elif len(payload) == CONTROL_TELEMETRY_PRE_TASKS_FRAME.size:
        (
            running,
            map_slot,
            target_index,
            point_count,
            speed,
            target_x,
            target_y,
            distance,
            error,
            steer,
            kp,
            ki,
            kd,
            motor_limit,
            loop_hz,
            mode,
            speed_profile_enabled,
            aux_percent,
            active_aux_percent,
            average_speed_mps,
            max_speed_mps,
            battery_compensation_enabled,
            control_map_x,
            control_map_y,
            control_map_heading,
            race_segment_active,
            race_segment_type,
            race_segment_start,
            race_segment_end,
            race_segment_speed,
            race_segment_max_speed,
            race_segment_aux,
            active_speed_percent,
            race_plan_average_speed_mps,
        ) = CONTROL_TELEMETRY_PRE_TASKS_FRAME.unpack(payload)
        control_map_pose_valid = True
        active_speed_received = True
    elif len(payload) == CONTROL_TELEMETRY_PRE_RACE_AVERAGE_FRAME.size:
        (
            running,
            map_slot,
            target_index,
            point_count,
            speed,
            target_x,
            target_y,
            distance,
            error,
            steer,
            kp,
            ki,
            kd,
            motor_limit,
            loop_hz,
            mode,
            speed_profile_enabled,
            aux_percent,
            active_aux_percent,
            average_speed_mps,
            max_speed_mps,
            battery_compensation_enabled,
            control_map_x,
            control_map_y,
            control_map_heading,
            race_segment_active,
            race_segment_type,
            race_segment_start,
            race_segment_end,
            race_segment_speed,
            race_segment_max_speed,
            race_segment_aux,
            active_speed_percent,
        ) = CONTROL_TELEMETRY_PRE_RACE_AVERAGE_FRAME.unpack(payload)
        control_map_pose_valid = True
        active_speed_received = True
    elif len(payload) == CONTROL_TELEMETRY_PRE_ACTIVE_SPEED_FRAME.size:
        (
            running,
            map_slot,
            target_index,
            point_count,
            speed,
            target_x,
            target_y,
            distance,
            error,
            steer,
            kp,
            ki,
            kd,
            motor_limit,
            loop_hz,
            mode,
            speed_profile_enabled,
            aux_percent,
            active_aux_percent,
            average_speed_mps,
            max_speed_mps,
            battery_compensation_enabled,
            control_map_x,
            control_map_y,
            control_map_heading,
            race_segment_active,
            race_segment_type,
            race_segment_start,
            race_segment_end,
            race_segment_speed,
            race_segment_max_speed,
            race_segment_aux,
        ) = CONTROL_TELEMETRY_PRE_ACTIVE_SPEED_FRAME.unpack(payload)
        active_speed_percent = abs(int(speed))
        control_map_pose_valid = True
    elif len(payload) == CONTROL_TELEMETRY_PRE_RACE_SEGMENT_FRAME.size:
        (
            running,
            map_slot,
            target_index,
            point_count,
            speed,
            target_x,
            target_y,
            distance,
            error,
            steer,
            kp,
            ki,
            kd,
            motor_limit,
            loop_hz,
            mode,
            speed_profile_enabled,
            aux_percent,
            active_aux_percent,
            average_speed_mps,
            max_speed_mps,
            battery_compensation_enabled,
            control_map_x,
            control_map_y,
            control_map_heading,
        ) = CONTROL_TELEMETRY_PRE_RACE_SEGMENT_FRAME.unpack(payload)
        control_map_pose_valid = True
    elif len(payload) == CONTROL_TELEMETRY_PRE_MAP_POSE_FRAME.size:
        (
            running,
            map_slot,
            target_index,
            point_count,
            speed,
            target_x,
            target_y,
            distance,
            error,
            steer,
            kp,
            ki,
            kd,
            motor_limit,
            loop_hz,
            mode,
            speed_profile_enabled,
            aux_percent,
            active_aux_percent,
            average_speed_mps,
            max_speed_mps,
            battery_compensation_enabled,
        ) = CONTROL_TELEMETRY_PRE_MAP_POSE_FRAME.unpack(payload)
    elif len(payload) == CONTROL_TELEMETRY_PRE_BATTERY_COMP_FRAME.size:
        (
            running,
            map_slot,
            target_index,
            point_count,
            speed,
            target_x,
            target_y,
            distance,
            error,
            steer,
            kp,
            ki,
            kd,
            motor_limit,
            loop_hz,
            mode,
            speed_profile_enabled,
            aux_percent,
            active_aux_percent,
            average_speed_mps,
            max_speed_mps,
        ) = CONTROL_TELEMETRY_PRE_BATTERY_COMP_FRAME.unpack(payload)
        battery_compensation_enabled = False
    elif len(payload) == CONTROL_TELEMETRY_PRE_SPEED_FRAME.size:
        (
            running,
            map_slot,
            target_index,
            point_count,
            speed,
            target_x,
            target_y,
            distance,
            error,
            steer,
            kp,
            ki,
            kd,
            motor_limit,
            loop_hz,
            mode,
            speed_profile_enabled,
            aux_percent,
            active_aux_percent,
        ) = CONTROL_TELEMETRY_PRE_SPEED_FRAME.unpack(payload)
        average_speed_mps = 0.0
        max_speed_mps = 0.0
        battery_compensation_enabled = False
    elif len(payload) == CONTROL_TELEMETRY_PRE_AUX_FRAME.size:
        (
            running,
            map_slot,
            target_index,
            point_count,
            speed,
            target_x,
            target_y,
            distance,
            error,
            steer,
            kp,
            ki,
            kd,
            motor_limit,
            loop_hz,
            mode,
            speed_profile_enabled,
        ) = CONTROL_TELEMETRY_PRE_AUX_FRAME.unpack(payload)
        aux_percent = 0
        active_aux_percent = 0
        average_speed_mps = 0.0
        max_speed_mps = 0.0
        battery_compensation_enabled = False
    elif len(payload) == CONTROL_TELEMETRY_PRE_PROFILE_FRAME.size:
        (
            running,
            map_slot,
            target_index,
            point_count,
            speed,
            target_x,
            target_y,
            distance,
            error,
            steer,
            kp,
            ki,
            kd,
            motor_limit,
            loop_hz,
            mode,
        ) = CONTROL_TELEMETRY_PRE_PROFILE_FRAME.unpack(payload)
        speed_profile_enabled = True
        aux_percent = 0
        active_aux_percent = 0
        average_speed_mps = 0.0
        max_speed_mps = 0.0
        battery_compensation_enabled = False
    elif len(payload) == CONTROL_TELEMETRY_PRE_MODE_FRAME.size:
        (
            running,
            map_slot,
            target_index,
            point_count,
            speed,
            target_x,
            target_y,
            distance,
            error,
            steer,
            kp,
            ki,
            kd,
            motor_limit,
            loop_hz,
        ) = CONTROL_TELEMETRY_PRE_MODE_FRAME.unpack(payload)
        mode = CONTROL_MODE_ODOMETRY
        speed_profile_enabled = True
        aux_percent = 0
        active_aux_percent = 0
        average_speed_mps = 0.0
        max_speed_mps = 0.0
        battery_compensation_enabled = False
    elif len(payload) == CONTROL_TELEMETRY_LEGACY_FRAME.size:
        (
            running,
            map_slot,
            target_index,
            point_count,
            speed,
            target_x,
            target_y,
            distance,
            error,
            steer,
            kp,
            ki,
            kd,
            motor_limit,
        ) = CONTROL_TELEMETRY_LEGACY_FRAME.unpack(payload)
        loop_hz = 0.0
        mode = CONTROL_MODE_ODOMETRY
        speed_profile_enabled = True
        aux_percent = 0
        active_aux_percent = 0
        average_speed_mps = 0.0
        max_speed_mps = 0.0
        battery_compensation_enabled = False
    else:
        raise ValueError("payload controle com tamanho invalido")

    if not active_speed_received:
        active_speed_percent = abs(int(speed))
    active_speed_percent = max(0, min(100, int(active_speed_percent)))

    return {
        "control_running": bool(running),
        "control_map_slot": map_slot,
        "control_target_index": target_index,
        "control_point_count": point_count,
        "control_speed_percent": speed,
        "control_active_speed_percent": active_speed_percent,
        "control_target_x_m": target_x,
        "control_target_y_m": target_y,
        "control_distance_m": distance,
        "control_angle_error_rad": error,
        "control_steer_percent": steer,
        "control_kp": kp,
        "control_ki": ki,
        "control_kd": kd,
        "control_motor_limit_percent": motor_limit,
        "control_loop_hz": loop_hz,
        "race_plan_loop_hz": race_plan_loop_hz,
        "track_odometry_loop_hz": track_odometry_loop_hz,
        "line_sensor_loop_hz": line_sensor_loop_hz,
        "imu_loop_hz": imu_loop_hz,
        "control_mode": mode,
        "control_speed_profile_enabled": bool(speed_profile_enabled),
        "control_aux_percent": aux_percent,
        "control_active_aux_percent": active_aux_percent,
        "control_average_speed_mps": average_speed_mps,
        "control_max_speed_mps": max_speed_mps,
        "control_race_plan_average_speed_mps": race_plan_average_speed_mps,
        "control_battery_compensation_enabled": bool(battery_compensation_enabled),
        "control_map_pose_valid": control_map_pose_valid,
        "control_map_x_m": control_map_x,
        "control_map_y_m": control_map_y,
        "control_map_heading_rad": control_map_heading,
        "control_race_segment_active": bool(race_segment_active),
        "control_race_segment_type": race_segment_type,
        "control_race_segment_start_index": race_segment_start,
        "control_race_segment_end_index": race_segment_end,
        "control_race_segment_speed_percent": race_segment_speed,
        "control_race_segment_max_speed_percent": race_segment_max_speed,
        "control_race_segment_aux_percent": race_segment_aux,
        "control_line_error_raw": line_error_raw,
        "control_line_error_normalized": line_error_normalized,
        "control_line_proportional_term": line_proportional_term,
        "control_line_nonlinear_term": line_nonlinear_term,
        "control_line_derivative_raw": line_derivative_raw,
        "control_line_derivative_filtered": line_derivative_filtered,
        "control_line_correction": line_correction,
        "control_line_left_command": line_left_command,
        "control_line_right_command": line_right_command,
        "control_line_dt_s": line_dt_s,
        "control_line_max_correction": line_max_correction,
        "control_line_derivative_filter_alpha": line_derivative_filter_alpha,
    }


def unpack_line_telemetry(payload: bytes) -> dict:
    if len(payload) == LINE_TELEMETRY_FRAME.size:
        values = LINE_TELEMETRY_FRAME.unpack(payload)
        threshold_percent = values[27]
        read_hz = values[28]
    elif len(payload) == LINE_TELEMETRY_PRE_HZ_FRAME.size:
        values = LINE_TELEMETRY_PRE_HZ_FRAME.unpack(payload)
        threshold_percent = values[27]
        read_hz = 0.0
    elif len(payload) == LINE_TELEMETRY_LEGACY_FRAME.size:
        values = LINE_TELEMETRY_LEGACY_FRAME.unpack(payload)
        threshold_percent = 0
        read_hz = 0.0
    else:
        raise ValueError("payload linha com tamanho invalido")

    raw = tuple(values[0:8])
    calibrated = tuple(values[8:16])
    line_values = tuple(values[16:24])
    position = values[24]
    track_type = values[25]
    flags = values[26]

    return {
        "line_raw": raw,
        "line_calibrated": calibrated,
        "line_values": line_values,
        "line_position": position,
        "line_track_type": track_type,
        "line_visible": bool(flags & (1 << 0)),
        "line_calibrated_valid": bool(flags & (1 << 1)),
        "line_calibrating": bool(flags & (1 << 2)),
        "line_threshold_percent": threshold_percent,
        "line_read_hz": read_hz,
    }


def unpack_line_fast_telemetry(payload: bytes) -> dict:
    if len(payload) != LINE_FAST_TELEMETRY_FRAME.size:
        raise ValueError("payload linha rapida com tamanho invalido")

    position, track_type, flags, threshold_percent, read_hz = LINE_FAST_TELEMETRY_FRAME.unpack(payload)
    return {
        "line_position": position,
        "line_track_type": track_type,
        "line_visible": bool(flags & (1 << 0)),
        "line_calibrated_valid": bool(flags & (1 << 1)),
        "line_calibrating": bool(flags & (1 << 2)),
        "line_threshold_percent": threshold_percent,
        "line_read_hz": read_hz,
    }


def unpack_rgb_led_telemetry(payload: bytes) -> dict:
    if len(payload) != RGB_LED_TELEMETRY_FRAME.size:
        raise ValueError("payload RGB LED com tamanho invalido")
    mode, red, green, blue, intensity, enabled = RGB_LED_TELEMETRY_FRAME.unpack(payload)
    return {
        "rgb_led_mode": mode,
        "rgb_led_red": red,
        "rgb_led_green": green,
        "rgb_led_blue": blue,
        "rgb_led_intensity": intensity,
        "rgb_led_enabled": bool(enabled),
    }


def unpack_safety_telemetry(payload: bytes) -> dict:
    if len(payload) == SAFETY_TELEMETRY_FRAME.size:
        flags, roll_limit, battery_limit, current_roll, current_battery, line_timeout, line_elapsed = (
            SAFETY_TELEMETRY_FRAME.unpack(payload)
        )
    elif len(payload) == SAFETY_TELEMETRY_U8_FLAGS_FRAME.size:
        flags, roll_limit, battery_limit, current_roll, current_battery, line_timeout, line_elapsed = (
            SAFETY_TELEMETRY_U8_FLAGS_FRAME.unpack(payload)
        )
    elif len(payload) == SAFETY_TELEMETRY_PRE_LINE_FRAME.size:
        flags, roll_limit, battery_limit, current_roll, current_battery = SAFETY_TELEMETRY_PRE_LINE_FRAME.unpack(payload)
        line_timeout = 1.0
        line_elapsed = 0.0
    else:
        raise ValueError("payload seguranca com tamanho invalido")
    return {
        "safety_collision_enabled": bool(flags & (1 << 0)),
        "safety_battery_block_enabled": bool(flags & (1 << 1)),
        "safety_collision_active": bool(flags & (1 << 2)),
        "safety_battery_block_active": bool(flags & (1 << 3)),
        "safety_motors_blocked": bool(flags & (1 << 4)),
        "safety_line_loss_enabled": bool(flags & (1 << 5)),
        "safety_line_loss_active": bool(flags & (1 << 6)),
        "safety_line_visible": bool(flags & (1 << 7)),
        "safety_ble_loss_enabled": bool(flags & (1 << 8)),
        "safety_ble_loss_active": bool(flags & (1 << 9)),
        "safety_ble_connected": bool(flags & (1 << 10)),
        "safety_roll_limit_deg": roll_limit,
        "safety_battery_block_percent": battery_limit,
        "safety_line_loss_timeout_s": line_timeout,
        "safety_line_loss_elapsed_s": line_elapsed,
        "safety_current_roll_deg": current_roll,
        "safety_current_battery_percent": current_battery,
    }


def unpack_system_telemetry(payload: bytes) -> dict:
    if len(payload) != SYSTEM_TELEMETRY_FRAME.size:
        raise ValueError("payload sistema com tamanho invalido")
    cpu0_percent, cpu1_percent = SYSTEM_TELEMETRY_FRAME.unpack(payload)
    return {
        "cpu0_percent": cpu0_percent,
        "cpu1_percent": cpu1_percent,
    }


def pack_pwm(value: int) -> bytes:
    return struct.pack("<b", max(-100, min(100, value)))
