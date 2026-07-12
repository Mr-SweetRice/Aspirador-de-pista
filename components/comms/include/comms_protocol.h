#ifndef COMMS_PROTOCOL_H
#define COMMS_PROTOCOL_H

#include <stdint.h>

#define COMMS_DEVICE_NAME "AspiradorPista-S3"
#define COMMS_AUTH_TOKEN "engineering-token"

#define COMMS_PROTOCOL_VERSION 1
#define COMMS_MAX_PAYLOAD_LEN 240

#define COMMS_SERVICE_UUID_STR "5d7a0000-8f5a-4a7d-9d4f-7a6c2b8d0001"
#define COMMS_AUTH_UUID_STR "5d7a0001-8f5a-4a7d-9d4f-7a6c2b8d0001"
#define COMMS_COMMAND_UUID_STR "5d7a0002-8f5a-4a7d-9d4f-7a6c2b8d0001"
#define COMMS_TELEMETRY_UUID_STR "5d7a0003-8f5a-4a7d-9d4f-7a6c2b8d0001"

#define COMMS_IMU_FLAG_MAG_CALIBRATING (1U << 0)
#define COMMS_IMU_FLAG_MAG_CALIBRATED (1U << 1)
#define COMMS_IMU_FLAG_ACCEL_GYRO_CALIBRATING (1U << 2)
#define COMMS_IMU_FLAG_ACCEL_GYRO_CALIBRATED (1U << 3)
#define COMMS_IMU_FLAG_MAG_IGNORED (1U << 4)
#define COMMS_IMU_FLAG_YAW_DRIFT_CALIBRATING (1U << 5)
#define COMMS_IMU_FLAG_YAW_DRIFT_CALIBRATED (1U << 6)

typedef enum {
    COMMS_CMD_CLASS_SAVE = 0x01,
    COMMS_CMD_CLASS_READ = 0x02,
    COMMS_CMD_CLASS_SEND = 0x03,
    COMMS_CMD_CLASS_TELE = 0x04,
} comms_command_class_t;

typedef enum {
    COMMS_ERROR_OK = 0x00,
    COMMS_ERROR_AUTH_REQUIRED = 0x01,
    COMMS_ERROR_AUTH_DENIED = 0x02,
    COMMS_ERROR_INVALID_PACKET = 0x03,
    COMMS_ERROR_UNSUPPORTED = 0x04,
    COMMS_ERROR_INTERNAL = 0x05,
} comms_error_t;

typedef enum {
    COMMS_SAVE_MAP_CHUNK = 0x01,
} comms_save_id_t;

typedef enum {
    COMMS_READ_MAP_LIST = 0x01,
    COMMS_READ_MAP_CHUNK = 0x02,
    COMMS_READ_MAP_RECORD_CHUNK = 0x03,
} comms_read_id_t;

typedef enum {
    COMMS_SEND_STOP = 0x01,
    COMMS_SEND_MOVE_FORWARD = 0x02,
    COMMS_SEND_MOVE_BACKWARD = 0x03,
    COMMS_SEND_SET_LEFT_PWM = 0x10,
    COMMS_SEND_SET_RIGHT_PWM = 0x11,
    COMMS_SEND_SET_AUX_PWM = 0x12,
    COMMS_SEND_RESET_ENCODERS = 0x20,
    COMMS_SEND_RESET_YAW = 0x21,
    COMMS_SEND_MAP_RECORD_START = 0x22,
    COMMS_SEND_MAP_RECORD_STOP = 0x23,
    COMMS_SEND_MAP_RECORD_SAVE = 0x24,
    COMMS_SEND_MAP_DELETE = 0x25,
    COMMS_SEND_CONTROL_START_MAP = 0x40,
    COMMS_SEND_CONTROL_STOP = 0x41,
    COMMS_SEND_CONTROL_SET_PID = 0x42,
    COMMS_SEND_ODOMETRY_SET_SOURCE = 0x43,
    COMMS_SEND_CONTROL_START_LINE = 0x44,
    COMMS_SEND_CONTROL_SET_SPEED_PROFILE = 0x45,
    COMMS_SEND_CONTROL_SAVE_PID = 0x46,
    COMMS_SEND_CONTROL_SET_SPEED_PROFILE_ENABLED = 0x47,
    COMMS_SEND_CONTROL_SET_AUX_PERCENT = 0x48,
    COMMS_SEND_ODOMETRY_SET_POSITION = 0x49,
    COMMS_SEND_CONTROL_SET_BATTERY_COMPENSATION_ENABLED = 0x4A,
    COMMS_SEND_CONTROL_START_AUTO_TRACK = 0x4B,
    COMMS_SEND_CONTROL_SET_AUTO_TRACK_CONFIG = 0x4C,
    COMMS_SEND_CONTROL_SET_RACE_PLAN = 0x4D,
    COMMS_SEND_IMU_CALIBRATE_MAG = 0x30,
    COMMS_SEND_IMU_CALIBRATE_ALL = 0x31,
    COMMS_SEND_IMU_CALIBRATE_ACCEL_GYRO = 0x32,
    COMMS_SEND_IMU_SET_MAG_FILTER_GAIN = 0x33,
    COMMS_SEND_IMU_SET_MAG_HEADING_MODE = 0x34,
    COMMS_SEND_IMU_SET_MAG_IGNORED = 0x35,
    COMMS_SEND_IMU_SET_YAW_DRIFT_THRESHOLD = 0x36,
    COMMS_SEND_IMU_CALIBRATE_YAW_DRIFT = 0x37,
    COMMS_SEND_LINE_CALIBRATE = 0x50,
    COMMS_SEND_LINE_SET_TRACK_TYPE = 0x51,
    COMMS_SEND_LINE_SET_THRESHOLD = 0x52,
    COMMS_SEND_RGB_LED_SET_ENABLED = 0x60,
    COMMS_SEND_RGB_LED_SET_MODE = 0x61,
    COMMS_SEND_RGB_LED_SET_MANUAL = 0x62,
    COMMS_SEND_SAFETY_SET_COLLISION_ENABLED = 0x70,
    COMMS_SEND_SAFETY_SET_BATTERY_BLOCK_ENABLED = 0x71,
    COMMS_SEND_SAFETY_SET_ROLL_LIMIT = 0x72,
    COMMS_SEND_SAFETY_SET_BATTERY_BLOCK_PERCENT = 0x73,
    COMMS_SEND_SAFETY_SET_LINE_LOSS_ENABLED = 0x74,
    COMMS_SEND_SAFETY_SET_LINE_LOSS_TIMEOUT = 0x75,
    COMMS_SEND_SAFETY_SET_BLE_LOSS_ENABLED = 0x76,
} comms_send_id_t;

typedef enum {
    COMMS_TELE_STATUS = 0x01,
    COMMS_TELE_ENCODERS = 0x02,
    COMMS_TELE_RPM = 0x03,
    COMMS_TELE_LINE = 0x04,
    COMMS_TELE_ODOMETRY = 0x05,
    COMMS_TELE_BATTERY = 0x06,
    COMMS_TELE_IMU = 0x07,
    COMMS_TELE_BUNDLE = 0x08,
    COMMS_TELE_POSE = 0x09,
    COMMS_TELE_LINE_FAST = 0x0A,
    COMMS_TELE_IMU_FAST = 0x0B,
    COMMS_TELE_MAP_LIST = 0x20,
    COMMS_TELE_MAP_CHUNK = 0x21,
    COMMS_TELE_MAP_RECORD_CHUNK = 0x22,
    COMMS_TELE_CONTROL = 0x30,
    COMMS_TELE_RGB_LED = 0x31,
    COMMS_TELE_SAFETY = 0x32,
    COMMS_TELE_SYSTEM = 0x33,
} comms_telemetry_id_t;

typedef struct {
    uint8_t version;
    uint8_t command_class;
    uint8_t message_id;
    uint8_t payload_len;
    const uint8_t *payload;
} comms_packet_view_t;

typedef struct __attribute__((packed)) {
    int32_t left_count;
    int32_t right_count;
    float left_rpm;
    float right_rpm;
    float linear_mps;
} comms_encoder_telemetry_payload_t;

typedef struct __attribute__((packed)) {
    float voltage_v;
    float percent;
    uint16_t raw;
} comms_battery_telemetry_payload_t;

typedef struct __attribute__((packed)) {
    float fused_x_m;
    float fused_y_m;
    float fused_heading_rad;
} comms_pose_telemetry_payload_t;

typedef struct __attribute__((packed)) {
    float x_m;
    float y_m;
    float heading_rad;
    float linear_mps;
    float angular_rad_s;
    float encoder_x_m;
    float encoder_y_m;
    float encoder_heading_rad;
    float imu_x_m;
    float imu_y_m;
    float imu_heading_rad;
    float fused_x_m;
    float fused_y_m;
    float fused_heading_rad;
    uint8_t imu_available;
} comms_odometry_telemetry_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t running;
    uint8_t map_slot;
    uint16_t target_index;
    uint16_t point_count;
    int8_t speed_percent;
    float target_x_m;
    float target_y_m;
    float distance_m;
    float angle_error_rad;
    float steer_percent;
    float kp;
    float ki;
    float kd;
    uint8_t motor_limit_percent;
    float loop_hz;
    float race_plan_loop_hz;
    float track_odometry_loop_hz;
    float line_sensor_loop_hz;
    float imu_loop_hz;
    uint8_t mode;
    uint8_t speed_profile_enabled;
    uint8_t aux_percent;
    uint8_t active_aux_percent;
    float average_speed_mps;
    float max_speed_mps;
    uint8_t battery_compensation_enabled;
    float map_x_m;
    float map_y_m;
    float map_heading_rad;
    uint8_t race_segment_active;
    uint8_t race_segment_type;
    uint16_t race_segment_start_index;
    uint16_t race_segment_end_index;
    uint8_t race_segment_speed_percent;
    uint8_t race_segment_max_speed_percent;
    uint8_t race_segment_aux_percent;
    uint8_t active_speed_percent;
    float race_plan_average_speed_mps;
} comms_control_telemetry_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t straight_speed_percent;
    uint8_t curve_speed_percent;
    float curve_angle_rad;
    uint8_t line_loss_odometry_enabled;
} comms_control_auto_track_config_payload_t;

typedef struct __attribute__((packed)) {
    uint16_t start_index;
    uint16_t end_index;
    uint8_t type;
    uint8_t speed_percent;
    uint8_t max_speed_percent;
    uint8_t aux_percent;
    float kp;
    float ki;
    float kd;
} comms_control_race_plan_segment_payload_t;

typedef struct __attribute__((packed)) {
    uint16_t raw[8];
    uint16_t calibrated[8];
    uint16_t line_values[8];
    uint16_t position;
    uint8_t track_type;
    uint8_t flags;
    uint8_t threshold_percent;
    float read_hz;
} comms_line_telemetry_payload_t;

typedef struct __attribute__((packed)) {
    uint16_t position;
    uint8_t track_type;
    uint8_t flags;
    uint8_t threshold_percent;
    float read_hz;
} comms_line_fast_telemetry_payload_t;

typedef struct __attribute__((packed)) {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float gyro_z_dps;
    float sample_hz;
    uint8_t mag_heading_mode;
    uint8_t flags;
} comms_imu_fast_telemetry_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t mode;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t intensity;
    uint8_t enabled;
} comms_rgb_led_telemetry_payload_t;

typedef struct __attribute__((packed)) {
    uint16_t flags;
    float roll_limit_deg;
    float battery_block_percent;
    float current_roll_deg;
    float current_battery_percent;
    float line_loss_timeout_s;
    float line_loss_elapsed_s;
} comms_safety_telemetry_payload_t;

typedef struct __attribute__((packed)) {
    float cpu0_percent;
    float cpu1_percent;
} comms_system_telemetry_payload_t;

typedef struct __attribute__((packed)) {
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float mag_yaw_deg;
    float mag_yaw_xy_deg;
    float mag_yaw_xz_deg;
    float mag_yaw_yz_deg;
    float mag_yaw_error_deg;
    float mag_field_norm_ut;
    float mag_filter_gain;
    float yaw_drift_threshold_dps;
    float quat_wxyz[4];
    float accel_mps2[3];
    float gyro_dps[3];
    float mag_ut[3];
    uint8_t mag_heading_mode;
    uint8_t flags;
} comms_imu_telemetry_payload_t;

#endif
