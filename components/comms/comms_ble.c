#include "comms_ble.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "comms_protocol.h"
#include "control.h"
#include "control_config.h"
#include "battery_level.h"
#include "encoder.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "host/ble_att.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "imu.h"
#include "line_sensor.h"
#include "memory_config.h"
#include "memory_maps.h"
#include "motors.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "odometry.h"
#include "rgb_led.h"
#include "safety.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h"

static const char *TAG = "comms_ble";

#define COMMS_BLE_TASK_CORE_ID 0
#define COMMS_BLE_TELEMETRY_TASK_PRIORITY 3
#define COMMS_BLE_TX_TASK_PRIORITY 4
#define COMMS_BLE_MAP_SAVE_TASK_PRIORITY 3
#define COMMS_BLE_SENSOR_TELEMETRY_TARGET_HZ 120
#define COMMS_BLE_REMAINING_TELEMETRY_TARGET_HZ 30
#define COMMS_BLE_SENSOR_TELEMETRY_PERIOD_US (1000000 / COMMS_BLE_SENSOR_TELEMETRY_TARGET_HZ)
#define COMMS_BLE_FULL_LINE_TELEMETRY_DIVIDER \
    (COMMS_BLE_SENSOR_TELEMETRY_TARGET_HZ / COMMS_BLE_REMAINING_TELEMETRY_TARGET_HZ)
#define COMMS_BLE_REGULAR_TELEMETRY_DIVIDER \
    (COMMS_BLE_SENSOR_TELEMETRY_TARGET_HZ / COMMS_BLE_REMAINING_TELEMETRY_TARGET_HZ)
#define COMMS_BLE_BUNDLE_NOTIFY_MIN_LEN 120
#define COMMS_BLE_DEFAULT_ATT_MTU 23
#define COMMS_BLE_ATT_NOTIFY_OVERHEAD 3
#define COMMS_BLE_TX_QUEUE_DEPTH 16
#define COMMS_BLE_NOTIFY_TX_TIMEOUT_MS 30
#define COMMS_BLE_NOTIFY_TX_BACKOFF_MS 100
#define COMMS_BLE_BULK_QUIET_MS 1200
#define COMMS_BLE_CONN_ITVL_MIN 12
#define COMMS_BLE_CONN_ITVL_MAX 24
#define COMMS_BLE_CONN_LATENCY 0
#define COMMS_BLE_CONN_TIMEOUT 600

#if COMMS_BLE_REMAINING_TELEMETRY_TARGET_HZ == 0 || \
    (COMMS_BLE_SENSOR_TELEMETRY_TARGET_HZ % COMMS_BLE_REMAINING_TELEMETRY_TARGET_HZ) != 0
#error "COMMS_BLE_SENSOR_TELEMETRY_TARGET_HZ deve ser multiplo de COMMS_BLE_REMAINING_TELEMETRY_TARGET_HZ"
#endif

static uint8_t own_addr_type;
static uint16_t active_conn_handle;
static uint16_t telemetry_value_handle;
static bool has_active_connection;
static bool is_authenticated;
static bool telemetry_subscribed;
static TaskHandle_t tx_task_handle;
static TaskHandle_t telemetry_task_handle;
static TaskHandle_t map_save_task_handle;
static QueueHandle_t tx_queue;
static esp_timer_handle_t telemetry_timer_handle;
static int64_t telemetry_quiet_until_us;
static int64_t auth_deadline_us;
static char pending_map_name[MEMORY_MAP_NAME_MAX_LEN + 1];
static memory_map_point_t pending_map_points[MEMORY_MAP_MAX_POINTS];
static bool pending_map_received[MEMORY_MAP_MAX_POINTS];
static uint16_t pending_map_total_points;
static uint16_t pending_map_received_points;
static uint16_t pending_map_next_offset;
static bool pending_map_active;
static char save_map_name[MEMORY_MAP_NAME_MAX_LEN + 1];
static memory_map_point_t save_map_points[MEMORY_MAP_MAX_POINTS];
static uint16_t save_map_total_points;
static bool save_map_pending;
static control_race_plan_segment_t pending_race_plan_segments[CONTROL_RACE_PLAN_MAX_SEGMENTS];
static uint8_t pending_race_plan_total;
static uint8_t pending_race_plan_next_offset;
static bool pending_race_plan_active;
static uint32_t notify_ok_count;
static uint32_t notify_fail_count;
static uint32_t command_write_count;
static uint32_t regular_telemetry_seq;
static uint32_t telemetry_tick_seq;
static uint16_t active_att_mtu = COMMS_BLE_DEFAULT_ATT_MTU;
static volatile bool notify_in_flight;
static int64_t telemetry_tx_backoff_until_us;
static configRUN_TIME_COUNTER_TYPE cpu_last_idle_runtime[2];
static int64_t cpu_last_sample_us;
static float cpu_usage_percent[2];

typedef struct {
    uint8_t packet[4 + COMMS_MAX_PAYLOAD_LEN];
    uint16_t len;
    char label[18];
} comms_ble_tx_item_t;

void ble_store_config_init(void);

static const ble_uuid128_t service_uuid =
    BLE_UUID128_INIT(0x01, 0x00, 0x8d, 0x2b, 0x6c, 0x7a, 0x4f, 0x9d,
                     0x7d, 0x4a, 0x5a, 0x8f, 0x00, 0x00, 0x7a, 0x5d);
static const ble_uuid128_t auth_uuid =
    BLE_UUID128_INIT(0x01, 0x00, 0x8d, 0x2b, 0x6c, 0x7a, 0x4f, 0x9d,
                     0x7d, 0x4a, 0x5a, 0x8f, 0x01, 0x00, 0x7a, 0x5d);
static const ble_uuid128_t command_uuid =
    BLE_UUID128_INIT(0x01, 0x00, 0x8d, 0x2b, 0x6c, 0x7a, 0x4f, 0x9d,
                     0x7d, 0x4a, 0x5a, 0x8f, 0x02, 0x00, 0x7a, 0x5d);
static const ble_uuid128_t telemetry_uuid =
    BLE_UUID128_INIT(0x01, 0x00, 0x8d, 0x2b, 0x6c, 0x7a, 0x4f, 0x9d,
                     0x7d, 0x4a, 0x5a, 0x8f, 0x03, 0x00, 0x7a, 0x5d);

static int comms_gap_event(struct ble_gap_event *event, void *arg);

static const char *gap_event_name(int event_type)
{
    switch (event_type) {
    case BLE_GAP_EVENT_CONNECT:
        return "CONNECT";
    case BLE_GAP_EVENT_DISCONNECT:
        return "DISCONNECT";
    case BLE_GAP_EVENT_CONN_UPDATE:
        return "CONN_UPDATE";
    case BLE_GAP_EVENT_SUBSCRIBE:
        return "SUBSCRIBE";
    case BLE_GAP_EVENT_MTU:
        return "MTU";
    case BLE_GAP_EVENT_NOTIFY_TX:
        return "NOTIFY_TX";
    case BLE_GAP_EVENT_ADV_COMPLETE:
        return "ADV_COMPLETE";
    default:
        return "OTHER";
    }
}

static const char *command_class_name(uint8_t command_class)
{
    switch (command_class) {
    case COMMS_CMD_CLASS_SAVE:
        return "SAVE";
    case COMMS_CMD_CLASS_READ:
        return "READ";
    case COMMS_CMD_CLASS_SEND:
        return "SEND";
    case COMMS_CMD_CLASS_TELE:
        return "TELE";
    default:
        return "UNKNOWN";
    }
}

static int copy_mbuf_payload(const struct os_mbuf *om,
                             uint8_t *buffer,
                             uint16_t buffer_len,
                             uint16_t *out_len)
{
    const uint16_t len = OS_MBUF_PKTLEN(om);

    if (len > buffer_len) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    if (os_mbuf_copydata(om, 0, len, buffer) != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    *out_len = len;
    return 0;
}

static bool parse_packet(const uint8_t *data, uint16_t len, comms_packet_view_t *packet)
{
    if (len < 4 || data[0] != COMMS_PROTOCOL_VERSION || data[3] != (len - 4)) {
        return false;
    }

    packet->version = data[0];
    packet->command_class = data[1];
    packet->message_id = data[2];
    packet->payload_len = data[3];
    packet->payload = &data[4];

    return true;
}

static bool append_telemetry_packet_to_bundle(uint8_t *bundle_payload,
                                              uint16_t *bundle_payload_len,
                                              uint16_t bundle_payload_max,
                                              const uint8_t *packet,
                                              uint16_t packet_len)
{
    if (packet_len < 4 || packet[0] != COMMS_PROTOCOL_VERSION || packet[1] != COMMS_CMD_CLASS_TELE ||
        packet[3] != packet_len - 4) {
        return false;
    }

    const uint16_t record_len = 2U + packet[3];
    if ((uint16_t)(*bundle_payload_len + record_len) > bundle_payload_max) {
        return false;
    }

    bundle_payload[(*bundle_payload_len)++] = packet[2];
    bundle_payload[(*bundle_payload_len)++] = packet[3];
    memcpy(&bundle_payload[*bundle_payload_len], &packet[4], packet[3]);
    *bundle_payload_len += packet[3];
    return true;
}

static uint16_t current_max_notify_len(void)
{
    return active_att_mtu > COMMS_BLE_ATT_NOTIFY_OVERHEAD ? active_att_mtu - COMMS_BLE_ATT_NOTIFY_OVERHEAD : 20;
}

static bool append_telemetry_packet_to_bundle_if_fits(uint8_t *bundle_payload,
                                                      uint16_t *bundle_payload_len,
                                                      uint16_t bundle_payload_max,
                                                      uint16_t max_notify_len,
                                                      const uint8_t *packet,
                                                      uint16_t packet_len)
{
    if (packet_len < 4 || packet[0] != COMMS_PROTOCOL_VERSION || packet[1] != COMMS_CMD_CLASS_TELE ||
        packet[3] != packet_len - 4) {
        return false;
    }

    const uint16_t record_len = 2U + packet[3];
    const uint16_t next_notify_len = 4U + *bundle_payload_len + record_len;
    if (next_notify_len > max_notify_len) {
        return false;
    }

    return append_telemetry_packet_to_bundle(bundle_payload, bundle_payload_len, bundle_payload_max, packet, packet_len);
}

static bool finish_bundle_packet(uint8_t *buffer,
                                 uint16_t buffer_len,
                                 uint16_t bundle_payload_len,
                                 uint16_t *out_len)
{
    if (bundle_payload_len == 0 || buffer_len < (uint16_t)(4U + bundle_payload_len)) {
        return false;
    }

    buffer[0] = COMMS_PROTOCOL_VERSION;
    buffer[1] = COMMS_CMD_CLASS_TELE;
    buffer[2] = COMMS_TELE_BUNDLE;
    buffer[3] = (uint8_t)bundle_payload_len;
    *out_len = 4U + bundle_payload_len;
    return true;
}

static void update_cpu_usage_sample(void)
{
    const int64_t now_us = esp_timer_get_time();
    if (cpu_last_sample_us == 0) {
        cpu_last_sample_us = now_us;
#if configGENERATE_RUN_TIME_STATS == 1
        cpu_last_idle_runtime[0] = ulTaskGetIdleRunTimeCounterForCore(0);
        cpu_last_idle_runtime[1] = ulTaskGetIdleRunTimeCounterForCore(1);
#endif
        return;
    }

    const int64_t elapsed_us = now_us - cpu_last_sample_us;
    if (elapsed_us <= 0) {
        return;
    }

#if configGENERATE_RUN_TIME_STATS == 1
    for (int core = 0; core < 2; ++core) {
        const configRUN_TIME_COUNTER_TYPE idle_runtime = ulTaskGetIdleRunTimeCounterForCore(core);
        const configRUN_TIME_COUNTER_TYPE idle_delta_runtime = idle_runtime - cpu_last_idle_runtime[core];
        float idle_percent = ((float)idle_delta_runtime * 100.0f) / (float)elapsed_us;
        if (idle_percent < 0.0f) {
            idle_percent = 0.0f;
        }
        if (idle_percent > 100.0f) {
            idle_percent = 100.0f;
        }
        cpu_usage_percent[core] = 100.0f - idle_percent;
        cpu_last_idle_runtime[core] = idle_runtime;
    }
#else
    cpu_usage_percent[0] = 0.0f;
    cpu_usage_percent[1] = 0.0f;
#endif
    cpu_last_sample_us = now_us;
}

static bool build_system_telemetry_packet(uint8_t *buffer, uint16_t buffer_len, uint16_t *out_len)
{
    if (buffer == NULL || out_len == NULL || buffer_len < 4 + sizeof(comms_system_telemetry_payload_t)) {
        return false;
    }

    update_cpu_usage_sample();

    const comms_system_telemetry_payload_t payload = {
        .cpu0_percent = cpu_usage_percent[0],
        .cpu1_percent = cpu_usage_percent[1],
    };

    buffer[0] = COMMS_PROTOCOL_VERSION;
    buffer[1] = COMMS_CMD_CLASS_TELE;
    buffer[2] = COMMS_TELE_SYSTEM;
    buffer[3] = sizeof(payload);
    memcpy(&buffer[4], &payload, sizeof(payload));
    *out_len = 4 + sizeof(payload);
    return true;
}

static bool build_imu_telemetry_packet(uint8_t *buffer, uint16_t buffer_len, uint16_t *out_len)
{
    imu_state_t imu = {0};
    comms_imu_telemetry_payload_t payload = {0};
    const uint16_t packet_len = 4 + sizeof(payload);

    if (buffer_len < packet_len || !imu_get_state(&imu)) {
        return false;
    }

    payload.roll_deg = imu.roll_deg;
    payload.pitch_deg = imu.pitch_deg;
    payload.yaw_deg = imu.yaw_deg;
    payload.mag_yaw_deg = imu.mag_yaw_deg;
    payload.mag_yaw_xy_deg = imu.mag_yaw_xy_deg;
    payload.mag_yaw_xz_deg = imu.mag_yaw_xz_deg;
    payload.mag_yaw_yz_deg = imu.mag_yaw_yz_deg;
    payload.mag_yaw_error_deg = imu.mag_yaw_error_deg;
    payload.mag_field_norm_ut = imu.mag_field_norm_ut;
    payload.mag_filter_gain = imu.mag_filter_gain;
    payload.yaw_drift_threshold_dps = imu.yaw_drift_threshold_dps;
    memcpy(payload.quat_wxyz, imu.quat_wxyz, sizeof(payload.quat_wxyz));
    memcpy(payload.accel_mps2, imu.accel_mps2, sizeof(payload.accel_mps2));
    memcpy(payload.gyro_dps, imu.gyro_dps, sizeof(payload.gyro_dps));
    memcpy(payload.mag_ut, imu.mag_ut, sizeof(payload.mag_ut));
    payload.mag_heading_mode = imu.mag_heading_mode;
    if (imu.mag_ignored) {
        payload.flags |= COMMS_IMU_FLAG_MAG_IGNORED;
    }
    if (imu.mag_calibrating) {
        payload.flags |= COMMS_IMU_FLAG_MAG_CALIBRATING;
    }
    if (imu.mag_calibrated) {
        payload.flags |= COMMS_IMU_FLAG_MAG_CALIBRATED;
    }
    if (imu.accel_gyro_calibrating) {
        payload.flags |= COMMS_IMU_FLAG_ACCEL_GYRO_CALIBRATING;
    }
    if (imu.accel_gyro_calibrated) {
        payload.flags |= COMMS_IMU_FLAG_ACCEL_GYRO_CALIBRATED;
    }
    if (imu.yaw_drift_calibrating) {
        payload.flags |= COMMS_IMU_FLAG_YAW_DRIFT_CALIBRATING;
    }
    if (imu.yaw_drift_calibrated) {
        payload.flags |= COMMS_IMU_FLAG_YAW_DRIFT_CALIBRATED;
    }

    buffer[0] = COMMS_PROTOCOL_VERSION;
    buffer[1] = COMMS_CMD_CLASS_TELE;
    buffer[2] = COMMS_TELE_IMU;
    buffer[3] = sizeof(payload);
    memcpy(&buffer[4], &payload, sizeof(payload));
    *out_len = packet_len;

    return true;
}

static bool build_imu_fast_telemetry_packet(uint8_t *buffer, uint16_t buffer_len, uint16_t *out_len)
{
    imu_state_t imu = {0};
    comms_imu_fast_telemetry_payload_t payload = {0};
    const uint16_t packet_len = 4 + sizeof(payload);

    if (buffer_len < packet_len || !imu_get_state(&imu)) {
        return false;
    }

    payload.roll_deg = imu.roll_deg;
    payload.pitch_deg = imu.pitch_deg;
    payload.yaw_deg = imu.yaw_deg;
    payload.gyro_z_dps = imu.gyro_dps[2];
    payload.sample_hz = imu.sample_hz;
    payload.mag_heading_mode = imu.mag_heading_mode;
    if (imu.mag_ignored) {
        payload.flags |= COMMS_IMU_FLAG_MAG_IGNORED;
    }
    if (imu.mag_calibrating) {
        payload.flags |= COMMS_IMU_FLAG_MAG_CALIBRATING;
    }
    if (imu.mag_calibrated) {
        payload.flags |= COMMS_IMU_FLAG_MAG_CALIBRATED;
    }
    if (imu.accel_gyro_calibrating) {
        payload.flags |= COMMS_IMU_FLAG_ACCEL_GYRO_CALIBRATING;
    }
    if (imu.accel_gyro_calibrated) {
        payload.flags |= COMMS_IMU_FLAG_ACCEL_GYRO_CALIBRATED;
    }
    if (imu.yaw_drift_calibrating) {
        payload.flags |= COMMS_IMU_FLAG_YAW_DRIFT_CALIBRATING;
    }
    if (imu.yaw_drift_calibrated) {
        payload.flags |= COMMS_IMU_FLAG_YAW_DRIFT_CALIBRATED;
    }

    buffer[0] = COMMS_PROTOCOL_VERSION;
    buffer[1] = COMMS_CMD_CLASS_TELE;
    buffer[2] = COMMS_TELE_IMU_FAST;
    buffer[3] = sizeof(payload);
    memcpy(&buffer[4], &payload, sizeof(payload));
    *out_len = packet_len;

    return true;
}

static bool build_encoder_telemetry_packet(uint8_t *buffer, uint16_t buffer_len, uint16_t *out_len)
{
    encoder_state_t encoder = {0};
    odometry_state_t odometry = {0};
    comms_encoder_telemetry_payload_t payload = {0};
    const uint16_t packet_len = 4 + sizeof(payload);

    if (buffer_len < packet_len || !encoder_get_state(&encoder)) {
        return false;
    }

    payload.left_count = encoder.counts[ENCODER_LEFT];
    payload.right_count = encoder.counts[ENCODER_RIGHT];
    payload.left_rpm = encoder.rpm[ENCODER_LEFT];
    payload.right_rpm = encoder.rpm[ENCODER_RIGHT];
    if (odometry_get_state(&odometry)) {
        payload.linear_mps = odometry.linear_mps;
    }

    buffer[0] = COMMS_PROTOCOL_VERSION;
    buffer[1] = COMMS_CMD_CLASS_TELE;
    buffer[2] = COMMS_TELE_ENCODERS;
    buffer[3] = sizeof(payload);
    memcpy(&buffer[4], &payload, sizeof(payload));
    *out_len = packet_len;

    return true;
}

static bool build_battery_telemetry_packet(uint8_t *buffer, uint16_t buffer_len, uint16_t *out_len)
{
    battery_level_state_t battery = {0};
    comms_battery_telemetry_payload_t payload = {0};
    const uint16_t packet_len = 4 + sizeof(payload);

    if (buffer_len < packet_len || !battery_level_get_state(&battery)) {
        return false;
    }

    payload.voltage_v = battery.voltage_v;
    payload.percent = battery.percent;
    payload.raw = battery.raw;

    buffer[0] = COMMS_PROTOCOL_VERSION;
    buffer[1] = COMMS_CMD_CLASS_TELE;
    buffer[2] = COMMS_TELE_BATTERY;
    buffer[3] = sizeof(payload);
    memcpy(&buffer[4], &payload, sizeof(payload));
    *out_len = packet_len;

    return true;
}

static bool build_odometry_telemetry_packet(uint8_t *buffer, uint16_t buffer_len, uint16_t *out_len)
{
    odometry_state_t odometry = {0};
    comms_odometry_telemetry_payload_t payload = {0};
    const uint16_t packet_len = 4 + sizeof(payload);

    if (buffer_len < packet_len || !odometry_get_state(&odometry)) {
        return false;
    }

    payload.x_m = odometry.x_m;
    payload.y_m = odometry.y_m;
    payload.heading_rad = odometry.heading_rad;
    payload.linear_mps = odometry.linear_mps;
    payload.angular_rad_s = odometry.angular_rad_s;
    payload.encoder_x_m = odometry.encoder_x_m;
    payload.encoder_y_m = odometry.encoder_y_m;
    payload.encoder_heading_rad = odometry.encoder_heading_rad;
    payload.imu_x_m = odometry.imu_x_m;
    payload.imu_y_m = odometry.imu_y_m;
    payload.imu_heading_rad = odometry.imu_heading_rad;
    payload.fused_x_m = odometry.fused_x_m;
    payload.fused_y_m = odometry.fused_y_m;
    payload.fused_heading_rad = odometry.fused_heading_rad;
    payload.imu_available = odometry.imu_available ? 1 : 0;

    buffer[0] = COMMS_PROTOCOL_VERSION;
    buffer[1] = COMMS_CMD_CLASS_TELE;
    buffer[2] = COMMS_TELE_ODOMETRY;
    buffer[3] = sizeof(payload);
    memcpy(&buffer[4], &payload, sizeof(payload));
    *out_len = packet_len;

    return true;
}

static bool build_pose_telemetry_packet(uint8_t *buffer, uint16_t buffer_len, uint16_t *out_len)
{
    odometry_state_t odometry = {0};
    comms_pose_telemetry_payload_t payload = {0};
    const uint16_t packet_len = 4 + sizeof(payload);

    if (buffer_len < packet_len || !odometry_get_state(&odometry)) {
        return false;
    }

    payload.fused_x_m = odometry.fused_x_m;
    payload.fused_y_m = odometry.fused_y_m;
    payload.fused_heading_rad = odometry.fused_heading_rad;

    buffer[0] = COMMS_PROTOCOL_VERSION;
    buffer[1] = COMMS_CMD_CLASS_TELE;
    buffer[2] = COMMS_TELE_POSE;
    buffer[3] = sizeof(payload);
    memcpy(&buffer[4], &payload, sizeof(payload));
    *out_len = packet_len;

    return true;
}

static bool build_control_telemetry_packet(uint8_t *buffer, uint16_t buffer_len, uint16_t *out_len)
{
    control_navigation_state_t control = {0};
    line_sensor_state_t line = {0};
    imu_state_t imu = {0};
    comms_control_telemetry_payload_t payload = {0};
    const uint16_t packet_len = 4 + sizeof(payload);

    if (buffer_len < packet_len || !control_get_navigation_state(&control)) {
        return false;
    }

    payload.running = control.running ? 1 : 0;
    payload.map_slot = control.map_slot;
    payload.target_index = control.target_index;
    payload.point_count = control.point_count;
    payload.speed_percent = control.speed_percent;
    payload.target_x_m = control.target_x_m;
    payload.target_y_m = control.target_y_m;
    payload.distance_m = control.distance_m;
    payload.angle_error_rad = control.angle_error_rad;
    payload.steer_percent = control.steer_percent;
    payload.kp = control.kp;
    payload.ki = control.ki;
    payload.kd = control.kd;
    payload.motor_limit_percent = control.motor_limit_percent;
    payload.loop_hz = control.loop_hz;
    payload.race_plan_loop_hz = control.race_plan_loop_hz;
    payload.track_odometry_loop_hz = control.track_odometry_loop_hz;
    payload.line_sensor_loop_hz = line_sensor_get_state(&line) ? line.read_hz : control.line_sensor_loop_hz;
    payload.imu_loop_hz = imu_get_state(&imu) ? imu.sample_hz : control.imu_loop_hz;
    payload.mode = (uint8_t)control.mode;
    payload.speed_profile_enabled = control.speed_profile_enabled ? 1 : 0;
    payload.aux_percent = control.aux_percent;
    payload.active_aux_percent = control.active_aux_percent;
    payload.average_speed_mps = control.average_speed_mps;
    payload.max_speed_mps = control.max_speed_mps;
    payload.battery_compensation_enabled = control.battery_compensation_enabled ? 1 : 0;
    payload.map_x_m = control.map_x_m;
    payload.map_y_m = control.map_y_m;
    payload.map_heading_rad = control.map_heading_rad;
    payload.race_segment_active = control.race_segment_active ? 1 : 0;
    payload.race_segment_type = control.race_segment_type;
    payload.race_segment_start_index = control.race_segment_start_index;
    payload.race_segment_end_index = control.race_segment_end_index;
    payload.race_segment_speed_percent = control.race_segment_speed_percent;
    payload.race_segment_max_speed_percent = control.race_segment_max_speed_percent;
    payload.race_segment_aux_percent = control.race_segment_aux_percent;
    payload.active_speed_percent = control.active_speed_percent;
    payload.race_plan_average_speed_mps = control.race_plan_average_speed_mps;
    payload.line_error_raw = control.line_error_raw;
    payload.line_error_normalized = control.line_error_normalized;
    payload.line_proportional_term = control.line_proportional_term;
    payload.line_nonlinear_term = control.line_nonlinear_term;
    payload.line_derivative_raw = control.line_derivative_raw;
    payload.line_derivative_filtered = control.line_derivative_filtered;
    payload.line_correction = control.line_correction;
    payload.line_left_command = control.line_left_command;
    payload.line_right_command = control.line_right_command;
    payload.line_dt_s = control.line_dt_s;
    payload.line_max_correction = control.line_max_correction;
    payload.line_derivative_filter_alpha = control.line_derivative_filter_alpha;

    buffer[0] = COMMS_PROTOCOL_VERSION;
    buffer[1] = COMMS_CMD_CLASS_TELE;
    buffer[2] = COMMS_TELE_CONTROL;
    buffer[3] = sizeof(payload);
    memcpy(&buffer[4], &payload, sizeof(payload));
    *out_len = packet_len;

    return true;
}

static bool build_line_telemetry_packet(uint8_t *buffer, uint16_t buffer_len, uint16_t *out_len)
{
    line_sensor_state_t line = {0};
    comms_line_telemetry_payload_t payload = {0};
    const uint16_t packet_len = 4 + sizeof(payload);

    if (buffer_len < packet_len || !line_sensor_get_state(&line)) {
        return false;
    }

    memcpy(payload.raw, line.raw, sizeof(payload.raw));
    memcpy(payload.calibrated, line.calibrated, sizeof(payload.calibrated));
    memcpy(payload.line_values, line.line_values, sizeof(payload.line_values));
    payload.position = line.position;
    payload.track_type = (uint8_t)line.track_type;
    payload.threshold_percent = line.threshold_percent;
    payload.read_hz = line.read_hz;
    if (line.line_visible) {
        payload.flags |= 1U << 0;
    }
    if (line.calibrated_valid) {
        payload.flags |= 1U << 1;
    }
    if (line.calibrating) {
        payload.flags |= 1U << 2;
    }

    buffer[0] = COMMS_PROTOCOL_VERSION;
    buffer[1] = COMMS_CMD_CLASS_TELE;
    buffer[2] = COMMS_TELE_LINE;
    buffer[3] = sizeof(payload);
    memcpy(&buffer[4], &payload, sizeof(payload));
    *out_len = packet_len;

    return true;
}

static bool build_line_fast_telemetry_packet(uint8_t *buffer, uint16_t buffer_len, uint16_t *out_len)
{
    line_sensor_state_t line = {0};
    comms_line_fast_telemetry_payload_t payload = {0};
    const uint16_t packet_len = 4 + sizeof(payload);

    if (buffer_len < packet_len || !line_sensor_get_state(&line)) {
        return false;
    }

    payload.position = line.position;
    payload.track_type = (uint8_t)line.track_type;
    payload.threshold_percent = line.threshold_percent;
    payload.read_hz = line.read_hz;
    if (line.line_visible) {
        payload.flags |= 1U << 0;
    }
    if (line.calibrated_valid) {
        payload.flags |= 1U << 1;
    }
    if (line.calibrating) {
        payload.flags |= 1U << 2;
    }

    buffer[0] = COMMS_PROTOCOL_VERSION;
    buffer[1] = COMMS_CMD_CLASS_TELE;
    buffer[2] = COMMS_TELE_LINE_FAST;
    buffer[3] = sizeof(payload);
    memcpy(&buffer[4], &payload, sizeof(payload));
    *out_len = packet_len;

    return true;
}

static bool build_rgb_led_telemetry_packet(uint8_t *buffer, uint16_t buffer_len, uint16_t *out_len)
{
    rgb_led_state_t led = {0};
    comms_rgb_led_telemetry_payload_t payload = {0};
    const uint16_t packet_len = 4 + sizeof(payload);

    if (buffer_len < packet_len || !rgb_led_get_state(&led)) {
        return false;
    }

    payload.mode = (uint8_t)led.mode;
    payload.red = led.red;
    payload.green = led.green;
    payload.blue = led.blue;
    payload.intensity = led.intensity;
    payload.enabled = led.enabled ? 1 : 0;

    buffer[0] = COMMS_PROTOCOL_VERSION;
    buffer[1] = COMMS_CMD_CLASS_TELE;
    buffer[2] = COMMS_TELE_RGB_LED;
    buffer[3] = sizeof(payload);
    memcpy(&buffer[4], &payload, sizeof(payload));
    *out_len = packet_len;

    return true;
}

static bool build_safety_telemetry_packet(uint8_t *buffer, uint16_t buffer_len, uint16_t *out_len)
{
    safety_state_t safety = {0};
    comms_safety_telemetry_payload_t payload = {0};
    const uint16_t packet_len = 4 + sizeof(payload);

    if (buffer_len < packet_len || !safety_get_state(&safety)) {
        return false;
    }

    if (safety.collision_enabled) {
        payload.flags |= 1U << 0;
    }
    if (safety.battery_block_enabled) {
        payload.flags |= 1U << 1;
    }
    if (safety.collision_active) {
        payload.flags |= 1U << 2;
    }
    if (safety.battery_block_active) {
        payload.flags |= 1U << 3;
    }
    if (safety.motors_blocked) {
        payload.flags |= 1U << 4;
    }
    if (safety.line_loss_enabled) {
        payload.flags |= 1U << 5;
    }
    if (safety.line_loss_active) {
        payload.flags |= 1U << 6;
    }
    if (safety.line_visible) {
        payload.flags |= 1U << 7;
    }
    if (safety.ble_loss_enabled) {
        payload.flags |= 1U << 8;
    }
    if (safety.ble_loss_active) {
        payload.flags |= 1U << 9;
    }
    if (safety.ble_connected) {
        payload.flags |= 1U << 10;
    }
    payload.roll_limit_deg = safety.roll_limit_deg;
    payload.battery_block_percent = safety.battery_block_percent;
    payload.current_roll_deg = safety.current_roll_deg;
    payload.current_battery_percent = safety.current_battery_percent;
    payload.line_loss_timeout_s = safety.line_loss_timeout_s;
    payload.line_loss_elapsed_s = safety.line_loss_elapsed_s;

    buffer[0] = COMMS_PROTOCOL_VERSION;
    buffer[1] = COMMS_CMD_CLASS_TELE;
    buffer[2] = COMMS_TELE_SAFETY;
    buffer[3] = sizeof(payload);
    memcpy(&buffer[4], &payload, sizeof(payload));
    *out_len = packet_len;

    return true;
}

static void build_status_packet(uint8_t *buffer, uint16_t *out_len, uint8_t status)
{
    buffer[0] = COMMS_PROTOCOL_VERSION;
    buffer[1] = COMMS_CMD_CLASS_TELE;
    buffer[2] = COMMS_TELE_STATUS;
    buffer[3] = 1;
    buffer[4] = status;
    *out_len = 5;
}

static bool can_notify_now(void)
{
    return has_active_connection && telemetry_subscribed && is_authenticated;
}

static bool tx_queue_idle(void)
{
    return tx_queue != NULL && !notify_in_flight && uxQueueMessagesWaiting(tx_queue) == 0;
}

static bool telemetry_tx_available(void)
{
    return can_notify_now() && tx_queue_idle() && esp_timer_get_time() >= telemetry_tx_backoff_until_us;
}

static bool is_priority_notify_label(const char *label)
{
    return strcmp(label, "status") == 0 ||
           strcmp(label, "map list") == 0 ||
           strcmp(label, "map chunk") == 0 ||
           strcmp(label, "map record chunk") == 0;
}

static int notify_packet_direct(const uint8_t *packet, uint16_t packet_len, const char *label)
{
    const uint16_t max_notify_len = current_max_notify_len();
    if (packet_len > max_notify_len) {
        ++notify_fail_count;
        ESP_LOGW(TAG,
                 "BLE notify ignorado label=%s len=%u mtu=%u max=%u",
                 label,
                 (unsigned int)packet_len,
                 (unsigned int)active_att_mtu,
                 (unsigned int)max_notify_len);
        return BLE_HS_EMSGSIZE;
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(packet, packet_len);
    if (om == NULL) {
        ESP_LOGW(TAG, "Falha ao alocar mbuf para telemetria %s", label);
        return BLE_HS_ENOMEM;
    }

    int rc = ble_gatts_notify_custom(active_conn_handle, telemetry_value_handle, om);
    if (rc != 0) {
        ++notify_fail_count;
        ESP_LOGW(TAG,
                 "BLE notify falhou label=%s len=%u rc=%d conn=%u active=%d sub=%d ok=%lu fail=%lu",
                 label,
                 (unsigned int)packet_len,
                 rc,
                 (unsigned int)active_conn_handle,
                 has_active_connection,
                 telemetry_subscribed,
                 (unsigned long)notify_ok_count,
                 (unsigned long)notify_fail_count);
    } else {
        ++notify_ok_count;
        if (strcmp(label, "map chunk") == 0 || strcmp(label, "map list") == 0 || strcmp(label, "status") == 0) {
            ESP_LOGI(TAG,
                     "BLE notify ok label=%s len=%u conn=%u ok=%lu fail=%lu",
                     label,
                     (unsigned int)packet_len,
                     (unsigned int)active_conn_handle,
                     (unsigned long)notify_ok_count,
                     (unsigned long)notify_fail_count);
        }
    }
    return rc;
}

static void notify_packet(const uint8_t *packet, uint16_t packet_len, const char *label)
{
    if (tx_queue == NULL || !can_notify_now()) {
        return;
    }

    comms_ble_tx_item_t item = {0};
    if (packet_len > sizeof(item.packet)) {
        ++notify_fail_count;
        ESP_LOGW(TAG, "BLE tx drop label=%s len=%u maior que buffer", label, (unsigned int)packet_len);
        return;
    }

    memcpy(item.packet, packet, packet_len);
    item.len = packet_len;
    strncpy(item.label, label, sizeof(item.label) - 1);

    const bool priority = is_priority_notify_label(label);
    if (!priority && !tx_queue_idle()) {
        return;
    }

    BaseType_t queued = priority
                            ? xQueueSendToFront(tx_queue, &item, 0)
                            : xQueueSend(tx_queue, &item, 0);
    if (queued != pdPASS) {
        comms_ble_tx_item_t dropped = {0};
        (void)xQueueReceive(tx_queue, &dropped, 0);
        queued = priority ? xQueueSendToFront(tx_queue, &item, 0) : xQueueSend(tx_queue, &item, 0);
    }

    if (queued != pdPASS) {
        ++notify_fail_count;
        ESP_LOGW(TAG, "BLE tx fila cheia label=%s len=%u", label, (unsigned int)packet_len);
    }
}

static void tx_task(void *param)
{
    (void)param;

    while (true) {
        comms_ble_tx_item_t item = {0};
        if (xQueueReceive(tx_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (!can_notify_now()) {
            continue;
        }

        (void)ulTaskNotifyTake(pdTRUE, 0);
        notify_in_flight = true;
        int rc = notify_packet_direct(item.packet, item.len, item.label);
        if (rc == 0) {
            const BaseType_t confirmed = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(COMMS_BLE_NOTIFY_TX_TIMEOUT_MS));
            if (confirmed == 0) {
                telemetry_tx_backoff_until_us =
                    esp_timer_get_time() + ((int64_t)COMMS_BLE_NOTIFY_TX_BACKOFF_MS * 1000);
            }
            notify_in_flight = false;
        } else {
            notify_in_flight = false;
            telemetry_tx_backoff_until_us =
                esp_timer_get_time() + ((int64_t)COMMS_BLE_NOTIFY_TX_BACKOFF_MS * 1000);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

static void pause_regular_telemetry(uint32_t duration_ms)
{
    const int64_t until_us = esp_timer_get_time() + ((int64_t)duration_ms * 1000);
    if (until_us > telemetry_quiet_until_us) {
        telemetry_quiet_until_us = until_us;
    }
}

static void start_bulk_tx_window(uint32_t duration_ms)
{
    pause_regular_telemetry(duration_ms);
    if (tx_queue != NULL) {
        xQueueReset(tx_queue);
    }
}

static void notify_status(uint8_t status)
{
    uint8_t packet[5] = {0};
    uint16_t packet_len = 0;

    if (!has_active_connection || !telemetry_subscribed || !is_authenticated) {
        return;
    }

    build_status_packet(packet, &packet_len, status);
    notify_packet(packet, packet_len, "status");
}

static void notify_map_list(void)
{
    start_bulk_tx_window(COMMS_BLE_BULK_QUIET_MS);

    uint8_t packet[4 + COMMS_MAX_PAYLOAD_LEN] = {0};
    memory_map_info_t maps[MEMORY_MAP_MAX_COUNT] = {0};
    size_t map_count = 0;
    esp_err_t ret = memory_maps_list(maps, MEMORY_MAP_MAX_COUNT, &map_count);

    if (ret != ESP_OK || !has_active_connection || !telemetry_subscribed) {
        notify_status(COMMS_ERROR_INTERNAL);
        return;
    }

    packet[0] = COMMS_PROTOCOL_VERSION;
    packet[1] = COMMS_CMD_CLASS_TELE;
    packet[2] = COMMS_TELE_MAP_LIST;
    packet[4] = (uint8_t)map_count;

    uint16_t offset = 5;
    for (size_t i = 0; i < map_count; ++i) {
        packet[offset++] = maps[i].slot;
        memcpy(&packet[offset], &maps[i].point_count, sizeof(maps[i].point_count));
        offset += sizeof(maps[i].point_count);
        memcpy(&packet[offset], &maps[i].distance_m, sizeof(maps[i].distance_m));
        offset += sizeof(maps[i].distance_m);
        memcpy(&packet[offset], maps[i].name, MEMORY_MAP_NAME_MAX_LEN + 1);
        offset += MEMORY_MAP_NAME_MAX_LEN + 1;
    }
    packet[3] = offset - 4;
    ESP_LOGI(TAG, "BLE map list count=%u payload=%u", (unsigned int)map_count, (unsigned int)packet[3]);
    notify_packet(packet, offset, "map list");
}

static void notify_map_chunk(uint8_t slot, uint16_t requested_offset)
{
    start_bulk_tx_window(COMMS_BLE_BULK_QUIET_MS);

    uint8_t packet[4 + COMMS_MAX_PAYLOAD_LEN] = {0};
    memory_map_point_t points[MEMORY_MAP_CHUNK_MAX_POINTS] = {0};
    uint16_t total_points = 0;
    uint8_t point_count = 0;
    esp_err_t ret = memory_maps_load_chunk(slot,
                                           requested_offset,
                                           points,
                                           MEMORY_MAP_CHUNK_MAX_POINTS,
                                           &total_points,
                                           &point_count);
    if (ret != ESP_OK || !has_active_connection || !telemetry_subscribed) {
        ESP_LOGW(TAG,
                 "BLE map chunk falhou slot=%u offset=%u ret=%s active=%d sub=%d",
                 (unsigned int)slot,
                 (unsigned int)requested_offset,
                 esp_err_to_name(ret),
                 has_active_connection,
                 telemetry_subscribed);
        notify_status(COMMS_ERROR_INTERNAL);
        return;
    }

    packet[0] = COMMS_PROTOCOL_VERSION;
    packet[1] = COMMS_CMD_CLASS_TELE;
    packet[2] = COMMS_TELE_MAP_CHUNK;
    packet[4] = slot;
    memcpy(&packet[5], &total_points, sizeof(total_points));
    memcpy(&packet[7], &requested_offset, sizeof(requested_offset));
    packet[9] = point_count;
    memcpy(&packet[10], points, point_count * sizeof(memory_map_point_t));
    packet[3] = 6 + (point_count * sizeof(memory_map_point_t));
    ESP_LOGI(TAG,
             "BLE map chunk tx slot=%u req_offset=%u total=%u count=%u len=%u",
             (unsigned int)slot,
             (unsigned int)requested_offset,
             (unsigned int)total_points,
             (unsigned int)point_count,
             (unsigned int)(10 + (point_count * sizeof(memory_map_point_t))));
    notify_packet(packet, 10 + (point_count * sizeof(memory_map_point_t)), "map chunk");
}

static void notify_map_record_chunk(uint16_t requested_offset)
{
    start_bulk_tx_window(COMMS_BLE_BULK_QUIET_MS);

    uint8_t packet[4 + COMMS_MAX_PAYLOAD_LEN] = {0};
    memory_map_point_t points[MEMORY_MAP_CHUNK_MAX_POINTS] = {0};
    uint16_t total_points = 0;
    uint8_t point_count = 0;
    memory_map_record_state_t record_state = {0};
    esp_err_t ret = memory_maps_record_load_chunk(requested_offset,
                                                  points,
                                                  MEMORY_MAP_CHUNK_MAX_POINTS,
                                                  &total_points,
                                                  &point_count,
                                                  &record_state);
    if (ret != ESP_OK || !has_active_connection || !telemetry_subscribed) {
        ESP_LOGW(TAG,
                 "BLE map record chunk falhou offset=%u ret=%s active=%d sub=%d",
                 (unsigned int)requested_offset,
                 esp_err_to_name(ret),
                 has_active_connection,
                 telemetry_subscribed);
        notify_status(ret == ESP_OK ? COMMS_ERROR_INTERNAL : COMMS_ERROR_INVALID_PACKET);
        return;
    }

    packet[0] = COMMS_PROTOCOL_VERSION;
    packet[1] = COMMS_CMD_CLASS_TELE;
    packet[2] = COMMS_TELE_MAP_RECORD_CHUNK;
    memcpy(&packet[4], &total_points, sizeof(total_points));
    memcpy(&packet[6], &requested_offset, sizeof(requested_offset));
    packet[8] = point_count;
    packet[9] = record_state.active ? 1U : 0U;
    memcpy(&packet[10], &record_state.rejected_points, sizeof(record_state.rejected_points));
    memcpy(&packet[12], &record_state.distance_m, sizeof(record_state.distance_m));
    memcpy(&packet[16], points, point_count * sizeof(memory_map_point_t));
    packet[3] = 12 + (point_count * sizeof(memory_map_point_t));

    ESP_LOGI(TAG,
             "BLE map record tx req_offset=%u total=%u count=%u active=%u rejected=%u distance=%.3f",
             (unsigned int)requested_offset,
             (unsigned int)total_points,
             (unsigned int)point_count,
             record_state.active ? 1U : 0U,
             (unsigned int)record_state.rejected_points,
             record_state.distance_m);
    notify_packet(packet, 16 + (point_count * sizeof(memory_map_point_t)), "map record chunk");
}

static void append_regular_telemetry_slot_to_bundle(uint8_t *bundle_payload,
                                                    uint16_t *bundle_payload_len,
                                                    uint16_t max_notify_len)
{
    uint8_t item[4 + COMMS_MAX_PAYLOAD_LEN] = {0};
    uint16_t packet_len = 0;

    const uint32_t slot = regular_telemetry_seq++ % 4U;
    switch (slot) {
    case 0:
        if (build_control_telemetry_packet(item, sizeof(item), &packet_len)) {
            append_telemetry_packet_to_bundle_if_fits(bundle_payload,
                                                      bundle_payload_len,
                                                      COMMS_MAX_PAYLOAD_LEN,
                                                      max_notify_len,
                                                      item,
                                                      packet_len);
        }
        if (build_system_telemetry_packet(item, sizeof(item), &packet_len)) {
            append_telemetry_packet_to_bundle_if_fits(bundle_payload,
                                                      bundle_payload_len,
                                                      COMMS_MAX_PAYLOAD_LEN,
                                                      max_notify_len,
                                                      item,
                                                      packet_len);
        }
        break;
    case 1:
        if (build_odometry_telemetry_packet(item, sizeof(item), &packet_len)) {
            append_telemetry_packet_to_bundle_if_fits(bundle_payload,
                                                      bundle_payload_len,
                                                      COMMS_MAX_PAYLOAD_LEN,
                                                      max_notify_len,
                                                      item,
                                                      packet_len);
        }
        if (build_encoder_telemetry_packet(item, sizeof(item), &packet_len)) {
            append_telemetry_packet_to_bundle_if_fits(bundle_payload,
                                                      bundle_payload_len,
                                                      COMMS_MAX_PAYLOAD_LEN,
                                                      max_notify_len,
                                                      item,
                                                      packet_len);
        }
        if (build_battery_telemetry_packet(item, sizeof(item), &packet_len)) {
            append_telemetry_packet_to_bundle_if_fits(bundle_payload,
                                                      bundle_payload_len,
                                                      COMMS_MAX_PAYLOAD_LEN,
                                                      max_notify_len,
                                                      item,
                                                      packet_len);
        }
        break;
    case 2:
        if (build_imu_telemetry_packet(item, sizeof(item), &packet_len)) {
            append_telemetry_packet_to_bundle_if_fits(bundle_payload,
                                                      bundle_payload_len,
                                                      COMMS_MAX_PAYLOAD_LEN,
                                                      max_notify_len,
                                                      item,
                                                      packet_len);
        }
        break;
    default:
        if (build_safety_telemetry_packet(item, sizeof(item), &packet_len)) {
            append_telemetry_packet_to_bundle_if_fits(bundle_payload,
                                                      bundle_payload_len,
                                                      COMMS_MAX_PAYLOAD_LEN,
                                                      max_notify_len,
                                                      item,
                                                      packet_len);
        }
        if (build_rgb_led_telemetry_packet(item, sizeof(item), &packet_len)) {
            append_telemetry_packet_to_bundle_if_fits(bundle_payload,
                                                      bundle_payload_len,
                                                      COMMS_MAX_PAYLOAD_LEN,
                                                      max_notify_len,
                                                      item,
                                                      packet_len);
        }
        if (build_system_telemetry_packet(item, sizeof(item), &packet_len)) {
            append_telemetry_packet_to_bundle_if_fits(bundle_payload,
                                                      bundle_payload_len,
                                                      COMMS_MAX_PAYLOAD_LEN,
                                                      max_notify_len,
                                                      item,
                                                      packet_len);
        }
        break;
    }
}

static void send_high_rate_sensor_telemetry_notification(uint32_t tick)
{
    uint8_t packet[4 + COMMS_MAX_PAYLOAD_LEN] = {0};
    uint8_t item[4 + COMMS_MAX_PAYLOAD_LEN] = {0};
    uint16_t packet_len = 0;
    uint16_t bundle_payload_len = 0;

    if (!has_active_connection || !telemetry_subscribed || !is_authenticated) {
        return;
    }

    if (!telemetry_tx_available()) {
        return;
    }

    const bool regular_telemetry_allowed = esp_timer_get_time() >= telemetry_quiet_until_us;
    const uint16_t max_notify_len = current_max_notify_len();

    if (build_pose_telemetry_packet(item, sizeof(item), &packet_len)) {
        append_telemetry_packet_to_bundle_if_fits(&packet[4],
                                                  &bundle_payload_len,
                                                  COMMS_MAX_PAYLOAD_LEN,
                                                  max_notify_len,
                                                  item,
                                                  packet_len);
    }

    if (build_imu_fast_telemetry_packet(item, sizeof(item), &packet_len)) {
        append_telemetry_packet_to_bundle_if_fits(&packet[4],
                                                  &bundle_payload_len,
                                                  COMMS_MAX_PAYLOAD_LEN,
                                                  max_notify_len,
                                                  item,
                                                  packet_len);
    }

    if (build_line_fast_telemetry_packet(item, sizeof(item), &packet_len)) {
        append_telemetry_packet_to_bundle_if_fits(&packet[4],
                                                  &bundle_payload_len,
                                                  COMMS_MAX_PAYLOAD_LEN,
                                                  max_notify_len,
                                                  item,
                                                  packet_len);
    }

    if (regular_telemetry_allowed &&
        (tick % COMMS_BLE_FULL_LINE_TELEMETRY_DIVIDER) == 0U &&
        build_line_telemetry_packet(item, sizeof(item), &packet_len)) {
        append_telemetry_packet_to_bundle_if_fits(&packet[4],
                                                  &bundle_payload_len,
                                                  COMMS_MAX_PAYLOAD_LEN,
                                                  max_notify_len,
                                                  item,
                                                  packet_len);
    }

    if (regular_telemetry_allowed && (tick % COMMS_BLE_REGULAR_TELEMETRY_DIVIDER) == 0U) {
        append_regular_telemetry_slot_to_bundle(&packet[4], &bundle_payload_len, max_notify_len);
    }

    if (finish_bundle_packet(packet, sizeof(packet), bundle_payload_len, &packet_len)) {
        notify_packet(packet, packet_len, "sensor fast");
    } else if (build_pose_telemetry_packet(packet, sizeof(packet), &packet_len)) {
        notify_packet(packet, packet_len, "pose telemetry");
    }
}

static void enforce_auth_timeout(void)
{
    if (has_active_connection && !is_authenticated && auth_deadline_us > 0 &&
        esp_timer_get_time() > auth_deadline_us) {
        ESP_LOGW(TAG, "BLE autenticacao timeout; encerrando conn=%u", (unsigned int)active_conn_handle);
        auth_deadline_us = 0;
        ble_gap_terminate(active_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
}

static void telemetry_task(void *param)
{
    (void)param;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        enforce_auth_timeout();
        const uint32_t tick = telemetry_tick_seq++;
        send_high_rate_sensor_telemetry_notification(tick);
    }
}

static void map_save_task(void *param)
{
    (void)param;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (!save_map_pending) {
            continue;
        }

        pause_regular_telemetry(500);
        ESP_LOGI(TAG,
                 "BLE map save worker start name=%s total=%u",
                 save_map_name,
                 (unsigned int)save_map_total_points);
        esp_err_t ret = memory_maps_save(save_map_name, save_map_total_points, save_map_points);
        save_map_pending = false;
        ESP_LOGI(TAG, "BLE map save worker done ret=%s", esp_err_to_name(ret));
        pause_regular_telemetry(350);
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
    }
}

static void telemetry_timer_cb(void *arg)
{
    (void)arg;

    if (telemetry_task_handle != NULL) {
        xTaskNotifyGive(telemetry_task_handle);
    }
}

static void handle_save_command(const comms_packet_view_t *packet)
{
    pause_regular_telemetry(350);

    if (packet->message_id != COMMS_SAVE_MAP_CHUNK) {
        ESP_LOGI(TAG, "SAVE id=0x%02x payload_len=%u sem handler", packet->message_id, packet->payload_len);
        notify_status(COMMS_ERROR_UNSUPPORTED);
        return;
    }

    const uint8_t header_len = MEMORY_MAP_NAME_MAX_LEN + 1 + sizeof(uint16_t) + sizeof(uint16_t) + sizeof(uint8_t);
    if (packet->payload_len < header_len) {
        notify_status(COMMS_ERROR_INVALID_PACKET);
        return;
    }

    char name[MEMORY_MAP_NAME_MAX_LEN + 1] = {0};
    uint16_t total_points = 0;
    uint16_t offset = 0;
    uint8_t point_count = 0;
    memcpy(name, packet->payload, MEMORY_MAP_NAME_MAX_LEN + 1);
    memcpy(&total_points, &packet->payload[MEMORY_MAP_NAME_MAX_LEN + 1], sizeof(total_points));
    memcpy(&offset, &packet->payload[MEMORY_MAP_NAME_MAX_LEN + 1 + sizeof(total_points)], sizeof(offset));
    point_count = packet->payload[MEMORY_MAP_NAME_MAX_LEN + 1 + sizeof(total_points) + sizeof(offset)];

    if (packet->payload_len != header_len + (point_count * sizeof(memory_map_point_t))) {
        notify_status(COMMS_ERROR_INVALID_PACKET);
        return;
    }

    if (name[0] == '\0' || total_points == 0 || total_points > MEMORY_MAP_MAX_POINTS ||
        point_count == 0 || point_count > MEMORY_MAP_CHUNK_MAX_POINTS ||
        offset >= total_points || (uint32_t)offset + point_count > total_points) {
        notify_status(COMMS_ERROR_INVALID_PACKET);
        return;
    }

    if (offset == 0 || !pending_map_active ||
        pending_map_total_points != total_points ||
        strncmp(pending_map_name, name, sizeof(pending_map_name)) != 0) {
        memset(pending_map_name, 0, sizeof(pending_map_name));
        memset(pending_map_points, 0, sizeof(pending_map_points));
        memset(pending_map_received, 0, sizeof(pending_map_received));
        strncpy(pending_map_name, name, MEMORY_MAP_NAME_MAX_LEN);
        pending_map_total_points = total_points;
        pending_map_received_points = 0;
        pending_map_next_offset = 0;
        pending_map_active = true;
        ESP_LOGI(TAG,
                 "BLE map rx start name=%s total=%u",
                 pending_map_name,
                 (unsigned int)pending_map_total_points);
    }

    if (offset != pending_map_next_offset) {
        ESP_LOGW(TAG,
                 "BLE map rx fora de ordem name=%s offset=%u esperado=%u count=%u total=%u",
                 pending_map_name,
                 (unsigned int)offset,
                 (unsigned int)pending_map_next_offset,
                 (unsigned int)point_count,
                 (unsigned int)pending_map_total_points);
        pending_map_active = false;
        pending_map_total_points = 0;
        pending_map_received_points = 0;
        pending_map_next_offset = 0;
        memset(pending_map_received, 0, sizeof(pending_map_received));
        notify_status(COMMS_ERROR_INVALID_PACKET);
        return;
    }

    const memory_map_point_t *points = (const memory_map_point_t *)&packet->payload[header_len];
    for (uint8_t i = 0; i < point_count; ++i) {
        const uint16_t point_index = offset + i;
        pending_map_points[point_index] = points[i];
        if (!pending_map_received[point_index]) {
            pending_map_received[point_index] = true;
            ++pending_map_received_points;
        }
    }
    pending_map_next_offset = (uint16_t)(offset + point_count);

    if (pending_map_received_points < pending_map_total_points) {
        ESP_LOGI(TAG,
                 "BLE map rx chunk name=%s offset=%u count=%u received=%u/%u",
                 pending_map_name,
                 (unsigned int)offset,
                 (unsigned int)point_count,
                 (unsigned int)pending_map_received_points,
                 (unsigned int)pending_map_total_points);
        return;
    }

    ESP_LOGI(TAG,
             "BLE map rx complete name=%s total=%u enfileirando save",
             pending_map_name,
             (unsigned int)pending_map_total_points);
    if (map_save_task_handle == NULL || save_map_pending) {
        ESP_LOGW(TAG, "BLE map save ocupado task=%p pending=%d", map_save_task_handle, save_map_pending);
        notify_status(COMMS_ERROR_INTERNAL);
        return;
    }

    memset(save_map_name, 0, sizeof(save_map_name));
    strncpy(save_map_name, pending_map_name, MEMORY_MAP_NAME_MAX_LEN);
    memcpy(save_map_points, pending_map_points, pending_map_total_points * sizeof(memory_map_point_t));
    save_map_total_points = pending_map_total_points;
    save_map_pending = true;
    pending_map_active = false;

    ESP_LOGI(TAG,
             "BLE map save queued name=%s total=%u",
             save_map_name,
             (unsigned int)save_map_total_points);
    xTaskNotifyGive(map_save_task_handle);
}

static void handle_read_command(const comms_packet_view_t *packet)
{
    switch (packet->message_id) {
    case COMMS_READ_MAP_LIST:
        notify_map_list();
        break;
    case COMMS_READ_MAP_CHUNK: {
        uint8_t slot = 0;
        uint16_t offset = 0;
        if (packet->payload_len < sizeof(slot) + sizeof(offset)) {
            notify_status(COMMS_ERROR_INVALID_PACKET);
            return;
        }
        slot = packet->payload[0];
        memcpy(&offset, &packet->payload[1], sizeof(offset));
        notify_map_chunk(slot, offset);
        break;
    }
    case COMMS_READ_MAP_RECORD_CHUNK: {
        uint16_t offset = 0;
        if (packet->payload_len < sizeof(offset)) {
            notify_status(COMMS_ERROR_INVALID_PACKET);
            return;
        }
        memcpy(&offset, packet->payload, sizeof(offset));
        notify_map_record_chunk(offset);
        break;
    }
    default:
        ESP_LOGI(TAG, "READ id=0x%02x payload_len=%u sem handler", packet->message_id, packet->payload_len);
        notify_status(COMMS_ERROR_UNSUPPORTED);
        break;
    }
}

static void handle_send_command(const comms_packet_view_t *packet)
{
    switch (packet->message_id) {
    case COMMS_SEND_STOP: {
        esp_err_t ret = control_emergency_stop();
        ESP_LOGI(TAG, "SEND emergency stop ret=%s", esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_MOVE_FORWARD: {
        esp_err_t left_ret = motors_set_percent(MOTORS_MOTOR_LEFT, MOTORS_DEFAULT_SPEED_PERCENT);
        esp_err_t right_ret = motors_set_percent(MOTORS_MOTOR_RIGHT, MOTORS_DEFAULT_SPEED_PERCENT);
        ESP_LOGI(TAG,
                 "SEND move forward speed=%d ret L=%s R=%s",
                 MOTORS_DEFAULT_SPEED_PERCENT,
                 esp_err_to_name(left_ret),
                 esp_err_to_name(right_ret));
        break;
    }
    case COMMS_SEND_MOVE_BACKWARD: {
        esp_err_t left_ret = motors_set_percent(MOTORS_MOTOR_LEFT, -MOTORS_DEFAULT_SPEED_PERCENT);
        esp_err_t right_ret = motors_set_percent(MOTORS_MOTOR_RIGHT, -MOTORS_DEFAULT_SPEED_PERCENT);
        ESP_LOGI(TAG,
                 "SEND move backward speed=%d ret L=%s R=%s",
                 MOTORS_DEFAULT_SPEED_PERCENT,
                 esp_err_to_name(left_ret),
                 esp_err_to_name(right_ret));
        break;
    }
    case COMMS_SEND_SET_LEFT_PWM: {
        int8_t percent = 0;
        if (packet->payload_len >= sizeof(percent)) {
            memcpy(&percent, packet->payload, sizeof(percent));
        }
        esp_err_t ret = motors_set_percent(MOTORS_MOTOR_LEFT, percent);
        ESP_LOGI(TAG, "SEND left pwm=%d ret=%s", (int)percent, esp_err_to_name(ret));
        break;
    }
    case COMMS_SEND_SET_RIGHT_PWM: {
        int8_t percent = 0;
        if (packet->payload_len >= sizeof(percent)) {
            memcpy(&percent, packet->payload, sizeof(percent));
        }
        esp_err_t ret = motors_set_percent(MOTORS_MOTOR_RIGHT, percent);
        ESP_LOGI(TAG, "SEND right pwm=%d ret=%s", (int)percent, esp_err_to_name(ret));
        break;
    }
    case COMMS_SEND_SET_AUX_PWM: {
        int8_t percent = 0;
        if (packet->payload_len >= sizeof(percent)) {
            memcpy(&percent, packet->payload, sizeof(percent));
        }
        esp_err_t ret = motors_set_percent(MOTORS_MOTOR_AUX, percent);
        ESP_LOGI(TAG, "SEND aux pwm=%d ret=%s", (int)percent, esp_err_to_name(ret));
        break;
    }
    case COMMS_SEND_CONTROL_START_MAP: {
        uint8_t slot = 0;
        int8_t speed = 25;
        if (packet->payload_len >= sizeof(slot) + sizeof(speed)) {
            slot = packet->payload[0];
            memcpy(&speed, &packet->payload[1], sizeof(speed));
        }
        esp_err_t ret = control_start_map(slot, speed);
        ESP_LOGI(TAG,
                 "SEND control start map slot=%u speed=%d ret=%s",
                 (unsigned int)slot,
                 (int)speed,
                 esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_CONTROL_START_LINE: {
        int8_t speed = 25;
        if (packet->payload_len >= sizeof(speed)) {
            memcpy(&speed, packet->payload, sizeof(speed));
        }
        esp_err_t ret = control_start_line(speed);
        ESP_LOGI(TAG,
                 "SEND control start line speed=%d ret=%s",
                 (int)speed,
                 esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_CONTROL_START_AUTO_TRACK: {
        uint8_t slot = 0;
        int8_t speed = 25;
        if (packet->payload_len >= sizeof(slot) + sizeof(speed)) {
            slot = packet->payload[0];
            memcpy(&speed, &packet->payload[1], sizeof(speed));
        }
        esp_err_t ret = control_start_auto_track(slot, speed);
        ESP_LOGI(TAG,
                 "SEND control start auto track slot=%u speed=%d ret=%s",
                 (unsigned int)slot,
                 (int)speed,
                 esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_CONTROL_SET_AUTO_TRACK_CONFIG: {
        comms_control_auto_track_config_payload_t payload = {0};
        esp_err_t ret = ESP_ERR_INVALID_SIZE;
        const uint8_t old_payload_len = sizeof(payload) + sizeof(float) + sizeof(float);
        if (packet->payload_len == sizeof(payload) || packet->payload_len == old_payload_len) {
            memcpy(&payload, packet->payload, sizeof(payload));
            const control_auto_track_config_t config = {
                .line_loss_odometry_enabled = payload.line_loss_odometry_enabled,
            };
            ret = control_set_auto_track_config(&config);
        }
        ESP_LOGI(TAG,
                 "SEND control auto track config payload=%u ret=%s",
                 packet->payload_len,
                 esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_CONTROL_SET_RACE_PLAN: {
        esp_err_t ret = ESP_ERR_INVALID_SIZE;
        if (packet->payload_len >= 4) {
            const uint8_t enabled = packet->payload[0];
            const uint8_t total = packet->payload[1];
            const uint8_t offset = packet->payload[2];
            const uint8_t count = packet->payload[3];
            const uint16_t expected_len = 4U + ((uint16_t)count * sizeof(comms_control_race_plan_segment_payload_t));

            if (!enabled) {
                pending_race_plan_active = false;
                pending_race_plan_total = 0;
                pending_race_plan_next_offset = 0;
                memset(pending_race_plan_segments, 0, sizeof(pending_race_plan_segments));
                ret = control_set_race_plan(NULL, 0, false);
            } else if (packet->payload_len == expected_len &&
                       total > 0 &&
                       total <= CONTROL_RACE_PLAN_MAX_SEGMENTS &&
                       count > 0 &&
                       count <= CONTROL_RACE_PLAN_CHUNK_MAX_SEGMENTS &&
                       offset < total &&
                       (uint16_t)offset + count <= total) {
                if (offset == 0 || !pending_race_plan_active || pending_race_plan_total != total) {
                    memset(pending_race_plan_segments, 0, sizeof(pending_race_plan_segments));
                    pending_race_plan_total = total;
                    pending_race_plan_next_offset = 0;
                    pending_race_plan_active = true;
                }

                if (offset != pending_race_plan_next_offset) {
                    ESP_LOGW(TAG,
                             "Plano corrida fora de ordem offset=%u esperado=%u total=%u count=%u",
                             (unsigned int)offset,
                             (unsigned int)pending_race_plan_next_offset,
                             (unsigned int)total,
                             (unsigned int)count);
                    pending_race_plan_active = false;
                    pending_race_plan_total = 0;
                    pending_race_plan_next_offset = 0;
                    memset(pending_race_plan_segments, 0, sizeof(pending_race_plan_segments));
                    ret = ESP_ERR_INVALID_ARG;
                } else {
                    for (uint8_t i = 0; i < count; ++i) {
                        comms_control_race_plan_segment_payload_t segment = {0};
                        memcpy(&segment,
                               &packet->payload[4 + (i * sizeof(segment))],
                               sizeof(segment));
                        pending_race_plan_segments[offset + i] = (control_race_plan_segment_t){
                            .start_index = segment.start_index,
                            .end_index = segment.end_index,
                            .type = segment.type,
                            .speed_percent = segment.speed_percent,
                            .max_speed_percent = segment.max_speed_percent,
                            .aux_percent = segment.aux_percent,
                            .kp = segment.kp,
                            .ki = segment.ki,
                            .kd = segment.kd,
                        };
                    }
                    pending_race_plan_next_offset = (uint8_t)(offset + count);
                    ret = ESP_OK;
                    if (pending_race_plan_next_offset >= pending_race_plan_total) {
                        ret = control_set_race_plan(pending_race_plan_segments, pending_race_plan_total, true);
                        pending_race_plan_active = false;
                    }
                }
            } else {
                ret = ESP_ERR_INVALID_ARG;
            }

            ESP_LOGI(TAG,
                     "SEND plano corrida enabled=%u total=%u offset=%u count=%u ret=%s",
                     (unsigned int)enabled,
                     (unsigned int)total,
                     (unsigned int)offset,
                     (unsigned int)count,
                     esp_err_to_name(ret));
        }
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_CONTROL_SET_LINE_CONTROLLER: {
        comms_control_line_controller_payload_t payload = {0};
        esp_err_t ret = ESP_ERR_INVALID_SIZE;
        if (packet->payload_len == sizeof(payload)) {
            memcpy(&payload, packet->payload, sizeof(payload));
            ret = control_set_line_controller(payload.kp,
                                              payload.kn,
                                              payload.kd,
                                              payload.max_correction_percent,
                                              payload.base_speed_percent,
                                              payload.derivative_filter_alpha);
        }
        ESP_LOGI(TAG,
                 "SEND controle linha kp=%.3f kn=%.3f kd=%.3f max_corr=%.1f base=%.1f d_alpha=%.3f ret=%s",
                 payload.kp,
                 payload.kn,
                 payload.kd,
                 payload.max_correction_percent,
                 payload.base_speed_percent,
                 payload.derivative_filter_alpha,
                 esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_CONTROL_STOP: {
        esp_err_t ret = control_stop_navigation();
        ESP_LOGI(TAG, "SEND control stop ret=%s", esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_CONTROL_SET_PID: {
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        uint8_t motor_limit_percent = 100;
        if (packet->payload_len >= sizeof(kp) + sizeof(ki) + sizeof(kd)) {
            memcpy(&kp, packet->payload, sizeof(kp));
            memcpy(&ki, &packet->payload[sizeof(kp)], sizeof(ki));
            memcpy(&kd, &packet->payload[sizeof(kp) + sizeof(ki)], sizeof(kd));
        }
        if (packet->payload_len >= sizeof(kp) + sizeof(ki) + sizeof(kd) + sizeof(motor_limit_percent)) {
            memcpy(&motor_limit_percent, &packet->payload[sizeof(kp) + sizeof(ki) + sizeof(kd)], sizeof(motor_limit_percent));
        } else {
            control_navigation_state_t current = {0};
            if (control_get_navigation_state(&current)) {
                motor_limit_percent = current.motor_limit_percent;
            }
        }
        esp_err_t ret = control_set_pid(kp, ki, kd);
        if (ret == ESP_OK) {
            ret = control_set_motor_limit(motor_limit_percent);
        }
        ESP_LOGI(TAG,
                 "SEND control set pid kp=%.3f ki=%.3f kd=%.3f limit=%u ret=%s",
                 kp,
                 ki,
                 kd,
                 (unsigned int)motor_limit_percent,
                 esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_CONTROL_SAVE_PID: {
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
        float line_max_correction = 0.0f;
        float line_base_speed = 0.0f;
        float line_derivative_alpha = 0.0f;
        uint8_t motor_limit_percent = 100;
        uint8_t aux_percent = 0;
        esp_err_t ret = ESP_ERR_INVALID_SIZE;
        if (packet->payload_len >= sizeof(kp) + sizeof(ki) + sizeof(kd) + sizeof(motor_limit_percent)) {
            const uint16_t base_len = sizeof(kp) + sizeof(ki) + sizeof(kd) + sizeof(motor_limit_percent);
            const uint16_t aux_len = base_len + sizeof(aux_percent);
            const uint16_t line_len = aux_len + sizeof(line_max_correction) +
                                      sizeof(line_base_speed) +
                                      sizeof(line_derivative_alpha);
            memcpy(&kp, packet->payload, sizeof(kp));
            memcpy(&ki, &packet->payload[sizeof(kp)], sizeof(ki));
            memcpy(&kd, &packet->payload[sizeof(kp) + sizeof(ki)], sizeof(kd));
            memcpy(&motor_limit_percent, &packet->payload[sizeof(kp) + sizeof(ki) + sizeof(kd)], sizeof(motor_limit_percent));
            control_navigation_state_t current = {0};
            if (control_get_navigation_state(&current)) {
                aux_percent = current.aux_percent;
            }
            if (packet->payload_len >= sizeof(kp) + sizeof(ki) + sizeof(kd) + sizeof(motor_limit_percent) + sizeof(aux_percent)) {
                memcpy(&aux_percent,
                       &packet->payload[sizeof(kp) + sizeof(ki) + sizeof(kd) + sizeof(motor_limit_percent)],
                       sizeof(aux_percent));
            }
            if (packet->payload_len >= line_len) {
                memcpy(&line_max_correction,
                       &packet->payload[aux_len],
                       sizeof(line_max_correction));
                memcpy(&line_base_speed,
                       &packet->payload[aux_len + sizeof(line_max_correction)],
                       sizeof(line_base_speed));
                memcpy(&line_derivative_alpha,
                       &packet->payload[aux_len + sizeof(line_max_correction) + sizeof(line_base_speed)],
                       sizeof(line_derivative_alpha));
                ret = control_set_line_controller(kp,
                                                  ki,
                                                  kd,
                                                  line_max_correction,
                                                  line_base_speed,
                                                  line_derivative_alpha);
            } else {
                ret = ESP_OK;
            }
            if (ret == ESP_OK) {
                ret = control_save_pid_settings(kp, ki, kd, motor_limit_percent, aux_percent);
            }
        }
        ESP_LOGI(TAG,
                 "SEND control save pid kp=%.3f kn=%.3f kd=%.3f limit=%u aux=%u line_max=%.1f base=%.1f d_alpha=%.3f ret=%s",
                 kp,
                 ki,
                 kd,
                 (unsigned int)motor_limit_percent,
                 (unsigned int)aux_percent,
                 line_max_correction,
                 line_base_speed,
                 line_derivative_alpha,
                 esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_CONTROL_SET_SPEED_PROFILE: {
        control_speed_profile_point_t points[CONTROL_SPEED_PROFILE_POINTS] = {0};
        esp_err_t ret = ESP_ERR_INVALID_SIZE;
        if (packet->payload_len == sizeof(points)) {
            memcpy(points, packet->payload, sizeof(points));
            ret = control_set_speed_profile(points, CONTROL_SPEED_PROFILE_POINTS);
        }
        ESP_LOGI(TAG,
                 "SEND control speed profile payload=%u ret=%s",
                 packet->payload_len,
                 esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_CONTROL_SET_SPEED_PROFILE_ENABLED: {
        bool enabled = true;
        esp_err_t ret = ESP_ERR_INVALID_SIZE;
        if (packet->payload_len >= 1) {
            enabled = packet->payload[0] != 0;
            ret = control_set_speed_profile_enabled(enabled);
        }
        ESP_LOGI(TAG,
                 "SEND control speed profile enabled=%u ret=%s",
                 enabled ? 1U : 0U,
                 esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_CONTROL_SET_AUX_PERCENT: {
        uint8_t aux_percent = 0;
        esp_err_t ret = ESP_ERR_INVALID_SIZE;
        if (packet->payload_len >= sizeof(aux_percent)) {
            aux_percent = packet->payload[0];
            ret = control_set_aux_percent(aux_percent);
        }
        ESP_LOGI(TAG,
                 "SEND control aux=%u ret=%s",
                 (unsigned int)aux_percent,
                 esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_CONTROL_SET_BATTERY_COMPENSATION_ENABLED: {
        bool enabled = false;
        esp_err_t ret = ESP_ERR_INVALID_SIZE;
        if (packet->payload_len >= 1) {
            enabled = packet->payload[0] != 0;
            ret = control_set_battery_compensation_enabled(enabled);
        }
        ESP_LOGI(TAG,
                 "SEND control battery compensation enabled=%u ret=%s",
                 enabled ? 1U : 0U,
                 esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_ODOMETRY_SET_SOURCE: {
        uint8_t source = ODOMETRY_POSE_SOURCE_FUSED;
        if (packet->payload_len >= sizeof(source)) {
            source = packet->payload[0];
        }
        esp_err_t ret = odometry_set_pose_source((odometry_pose_source_t)source);
        ESP_LOGI(TAG,
                 "SEND odometry source=%u ret=%s",
                 (unsigned int)source,
                 esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_ODOMETRY_SET_POSITION: {
        float x_m = 0.0f;
        float y_m = 0.0f;
        esp_err_t ret = ESP_ERR_INVALID_SIZE;
        esp_err_t imu_ret = ESP_ERR_INVALID_STATE;
        esp_err_t heading_ret = ESP_ERR_INVALID_STATE;
        if (packet->payload_len >= sizeof(x_m) + sizeof(y_m)) {
            memcpy(&x_m, packet->payload, sizeof(x_m));
            memcpy(&y_m, &packet->payload[sizeof(x_m)], sizeof(y_m));
            imu_ret = imu_reset_yaw();
            heading_ret = odometry_reset_heading();
            ret = odometry_set_position(x_m, y_m);
            if (imu_ret != ESP_OK || heading_ret != ESP_OK) {
                ret = ret == ESP_OK ? ESP_ERR_INVALID_STATE : ret;
            }
        }
        ESP_LOGI(TAG,
                 "SEND odometry pose reset x=%.3f y=%.3f imu=%s heading=%s position=%s",
                 x_m,
                 y_m,
                 esp_err_to_name(imu_ret),
                 esp_err_to_name(heading_ret),
                 esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_RESET_ENCODERS:
    {
        esp_err_t encoder_ret = encoder_reset();
        esp_err_t odometry_ret = odometry_reset();
        ESP_LOGI(TAG,
                 "SEND reset encoders ret encoder=%s odometry=%s",
                 esp_err_to_name(encoder_ret),
                 esp_err_to_name(odometry_ret));
        break;
    }
    case COMMS_SEND_RESET_YAW: {
        esp_err_t imu_ret = imu_reset_yaw();
        esp_err_t odometry_ret = odometry_reset_heading();
        ESP_LOGI(TAG,
                 "SEND reset yaw ret imu=%s odometry=%s",
                 esp_err_to_name(imu_ret),
                 esp_err_to_name(odometry_ret));
        break;
    }
    case COMMS_SEND_MAP_RECORD_START: {
        char name[MEMORY_MAP_NAME_MAX_LEN + 1] = {0};
        if (packet->payload_len > 0) {
            const uint8_t copy_len = packet->payload_len > MEMORY_MAP_NAME_MAX_LEN ?
                                         MEMORY_MAP_NAME_MAX_LEN :
                                         packet->payload_len;
            memcpy(name, packet->payload, copy_len);
        }
        if (name[0] == '\0') {
            strncpy(name, "mapa", MEMORY_MAP_NAME_MAX_LEN);
        }
        esp_err_t imu_ret = imu_reset_yaw();
        esp_err_t odometry_ret = odometry_reset();
        esp_err_t ret = ESP_OK;
        if (imu_ret != ESP_OK || odometry_ret != ESP_OK) {
            ret = ESP_ERR_INVALID_STATE;
        } else {
            ret = memory_maps_record_start(name);
        }
        ESP_LOGI(TAG,
                 "SEND map record start name=%s imu=%s odometry=%s ret=%s",
                 name,
                 esp_err_to_name(imu_ret),
                 esp_err_to_name(odometry_ret),
                 esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_MAP_RECORD_STOP: {
        esp_err_t ret = memory_maps_record_stop();
        ESP_LOGI(TAG, "SEND map record stop ret=%s", esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_MAP_RECORD_SAVE: {
        char name[MEMORY_MAP_NAME_MAX_LEN + 1] = {0};
        if (packet->payload_len > 0) {
            const uint8_t copy_len = packet->payload_len > MEMORY_MAP_NAME_MAX_LEN ?
                                         MEMORY_MAP_NAME_MAX_LEN :
                                         packet->payload_len;
            memcpy(name, packet->payload, copy_len);
        }
        esp_err_t ret = memory_maps_record_save(name);
        ESP_LOGI(TAG, "SEND map record save name=%s ret=%s", name, esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_MAP_DELETE: {
        if (packet->payload_len < sizeof(uint8_t)) {
            notify_status(COMMS_ERROR_INVALID_PACKET);
            break;
        }
        const uint8_t slot = packet->payload[0];
        esp_err_t ret = memory_maps_delete(slot);
        ESP_LOGI(TAG, "SEND map delete slot=%u ret=%s", (unsigned int)slot, esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        if (ret == ESP_OK) {
            notify_map_list();
        }
        break;
    }
    case COMMS_SEND_IMU_CALIBRATE_MAG: {
        uint32_t duration_ms = 30000;
        if (packet->payload_len >= sizeof(duration_ms)) {
            memcpy(&duration_ms, packet->payload, sizeof(duration_ms));
        }
        esp_err_t ret = imu_start_mag_calibration(duration_ms);
        ESP_LOGI(TAG, "SEND IMU calibrate mag duration=%lu ret=%s", (unsigned long)duration_ms, esp_err_to_name(ret));
        break;
    }
    case COMMS_SEND_IMU_CALIBRATE_ALL: {
        uint32_t duration_ms = 45000;
        if (packet->payload_len >= sizeof(duration_ms)) {
            memcpy(&duration_ms, packet->payload, sizeof(duration_ms));
        }
        esp_err_t ret = imu_start_full_calibration(duration_ms);
        ESP_LOGI(TAG, "SEND IMU calibrate all duration=%lu ret=%s", (unsigned long)duration_ms, esp_err_to_name(ret));
        break;
    }
    case COMMS_SEND_IMU_CALIBRATE_ACCEL_GYRO: {
        uint32_t duration_ms = 5000;
        if (packet->payload_len >= sizeof(duration_ms)) {
            memcpy(&duration_ms, packet->payload, sizeof(duration_ms));
        }
        esp_err_t ret = imu_start_accel_gyro_calibration(duration_ms);
        ESP_LOGI(TAG, "SEND IMU calibrate accel/gyro duration=%lu ret=%s", (unsigned long)duration_ms, esp_err_to_name(ret));
        break;
    }
    case COMMS_SEND_IMU_CALIBRATE_YAW_DRIFT: {
        uint32_t duration_ms = 20000;
        if (packet->payload_len >= sizeof(duration_ms)) {
            memcpy(&duration_ms, packet->payload, sizeof(duration_ms));
        }
        esp_err_t ret = imu_start_yaw_drift_calibration(duration_ms);
        ESP_LOGI(TAG, "SEND IMU calibrate yaw drift duration=%lu ret=%s", (unsigned long)duration_ms, esp_err_to_name(ret));
        break;
    }
    case COMMS_SEND_IMU_SET_MAG_FILTER_GAIN: {
        float gain = imu_get_mag_filter_gain();
        if (packet->payload_len >= sizeof(gain)) {
            memcpy(&gain, packet->payload, sizeof(gain));
        }
        esp_err_t ret = imu_set_mag_filter_gain(gain);
        ESP_LOGI(TAG, "SEND IMU mag filter gain=%.4f ret=%s", gain, esp_err_to_name(ret));
        break;
    }
    case COMMS_SEND_IMU_SET_MAG_HEADING_MODE: {
        uint8_t mode = imu_get_mag_heading_mode();
        if (packet->payload_len >= sizeof(mode)) {
            memcpy(&mode, packet->payload, sizeof(mode));
        }
        esp_err_t ret = imu_set_mag_heading_mode(mode);
        ESP_LOGI(TAG, "SEND IMU mag heading mode=%u ret=%s", (unsigned int)mode, esp_err_to_name(ret));
        break;
    }
    case COMMS_SEND_IMU_SET_MAG_IGNORED: {
        uint8_t ignored = imu_get_mag_ignored() ? 1 : 0;
        if (packet->payload_len >= sizeof(ignored)) {
            memcpy(&ignored, packet->payload, sizeof(ignored));
        }
        esp_err_t ret = imu_set_mag_ignored(ignored != 0);
        ESP_LOGI(TAG, "SEND IMU mag ignored=%u ret=%s", (unsigned int)ignored, esp_err_to_name(ret));
        break;
    }
    case COMMS_SEND_IMU_SET_YAW_DRIFT_THRESHOLD: {
        float threshold = imu_get_yaw_drift_threshold();
        if (packet->payload_len >= sizeof(threshold)) {
            memcpy(&threshold, packet->payload, sizeof(threshold));
        }
        esp_err_t ret = imu_set_yaw_drift_threshold(threshold);
        ESP_LOGI(TAG, "SEND IMU yaw drift threshold=%.3f ret=%s", threshold, esp_err_to_name(ret));
        break;
    }
    case COMMS_SEND_LINE_CALIBRATE: {
        uint32_t duration_ms = 5000;
        if (packet->payload_len >= sizeof(duration_ms)) {
            memcpy(&duration_ms, packet->payload, sizeof(duration_ms));
        }
        esp_err_t ret = line_sensor_start_calibration(duration_ms);
        ESP_LOGI(TAG, "SEND line calibrate duration=%lu ret=%s", (unsigned long)duration_ms, esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_LINE_SET_TRACK_TYPE: {
        uint8_t track_type = (uint8_t)line_sensor_get_track_type();
        if (packet->payload_len >= sizeof(track_type)) {
            track_type = packet->payload[0];
        }
        esp_err_t ret = line_sensor_set_track_type((line_sensor_track_type_t)track_type);
        ESP_LOGI(TAG, "SEND line track type=%u ret=%s", (unsigned int)track_type, esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_LINE_SET_THRESHOLD: {
        uint8_t threshold_percent = 0;
        if (packet->payload_len >= sizeof(threshold_percent)) {
            threshold_percent = packet->payload[0];
        }
        esp_err_t ret = line_sensor_set_threshold_percent(threshold_percent);
        ESP_LOGI(TAG,
                 "SEND line threshold=%u%% ret=%s",
                 (unsigned int)threshold_percent,
                 esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_RGB_LED_SET_ENABLED: {
        uint8_t enabled = 1;
        if (packet->payload_len >= sizeof(enabled)) {
            enabled = packet->payload[0];
        }
        esp_err_t ret = rgb_led_set_enabled(enabled != 0);
        ESP_LOGI(TAG, "SEND rgb led enabled=%u ret=%s", (unsigned int)enabled, esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_RGB_LED_SET_MODE: {
        uint8_t mode = RGB_LED_MODE_BATTERY;
        if (packet->payload_len >= sizeof(mode)) {
            mode = packet->payload[0];
        }
        esp_err_t ret = rgb_led_set_mode((rgb_led_mode_t)mode);
        ESP_LOGI(TAG, "SEND rgb led mode=%u ret=%s", (unsigned int)mode, esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_RGB_LED_SET_MANUAL: {
        uint8_t values[4] = {255, 255, 255, 32};
        if (packet->payload_len >= sizeof(values)) {
            memcpy(values, packet->payload, sizeof(values));
        }
        esp_err_t ret = rgb_led_set_manual_color(values[0], values[1], values[2], values[3]);
        ESP_LOGI(TAG,
                 "SEND rgb led manual r=%u g=%u b=%u i=%u ret=%s",
                 values[0],
                 values[1],
                 values[2],
                 values[3],
                 esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_SAFETY_SET_COLLISION_ENABLED: {
        uint8_t enabled = 1;
        esp_err_t ret = ESP_ERR_INVALID_SIZE;
        if (packet->payload_len >= sizeof(enabled)) {
            enabled = packet->payload[0];
            ret = safety_set_collision_enabled(enabled != 0);
        }
        ESP_LOGI(TAG, "SEND safety collision enabled=%u ret=%s", (unsigned int)enabled, esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_SAFETY_SET_BATTERY_BLOCK_ENABLED: {
        uint8_t enabled = 1;
        esp_err_t ret = ESP_ERR_INVALID_SIZE;
        if (packet->payload_len >= sizeof(enabled)) {
            enabled = packet->payload[0];
            ret = safety_set_battery_block_enabled(enabled != 0);
        }
        ESP_LOGI(TAG, "SEND safety battery enabled=%u ret=%s", (unsigned int)enabled, esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_SAFETY_SET_LINE_LOSS_ENABLED: {
        uint8_t enabled = 1;
        esp_err_t ret = ESP_ERR_INVALID_SIZE;
        if (packet->payload_len >= sizeof(enabled)) {
            enabled = packet->payload[0];
            ret = safety_set_line_loss_enabled(enabled != 0);
        }
        ESP_LOGI(TAG, "SEND safety line enabled=%u ret=%s", (unsigned int)enabled, esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_SAFETY_SET_ROLL_LIMIT: {
        float limit = 6.0f;
        esp_err_t ret = ESP_ERR_INVALID_SIZE;
        if (packet->payload_len >= sizeof(limit)) {
            memcpy(&limit, packet->payload, sizeof(limit));
            ret = safety_set_roll_limit_deg(limit);
        }
        ESP_LOGI(TAG, "SEND safety roll limit=%.2f ret=%s", limit, esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_SAFETY_SET_BATTERY_BLOCK_PERCENT: {
        float percent = 10.0f;
        esp_err_t ret = ESP_ERR_INVALID_SIZE;
        if (packet->payload_len >= sizeof(percent)) {
            memcpy(&percent, packet->payload, sizeof(percent));
            ret = safety_set_battery_block_percent(percent);
        }
        ESP_LOGI(TAG, "SEND safety battery percent=%.2f ret=%s", percent, esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_SAFETY_SET_LINE_LOSS_TIMEOUT: {
        float timeout_s = 1.0f;
        esp_err_t ret = ESP_ERR_INVALID_SIZE;
        if (packet->payload_len >= sizeof(timeout_s)) {
            memcpy(&timeout_s, packet->payload, sizeof(timeout_s));
            ret = safety_set_line_loss_timeout_s(timeout_s);
        }
        ESP_LOGI(TAG, "SEND safety line timeout=%.2f ret=%s", timeout_s, esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    case COMMS_SEND_SAFETY_SET_BLE_LOSS_ENABLED: {
        uint8_t enabled = 1;
        esp_err_t ret = ESP_ERR_INVALID_SIZE;
        if (packet->payload_len >= sizeof(enabled)) {
            enabled = packet->payload[0];
            ret = safety_set_ble_loss_enabled(enabled != 0);
        }
        ESP_LOGI(TAG, "SEND safety ble enabled=%u ret=%s", (unsigned int)enabled, esp_err_to_name(ret));
        notify_status(ret == ESP_OK ? COMMS_ERROR_OK : COMMS_ERROR_INTERNAL);
        break;
    }
    default:
        ESP_LOGI(TAG,
                 "SEND id=0x%02x payload_len=%u sem handler",
                 packet->message_id,
                 packet->payload_len);
        break;
    }
}

static void handle_tele_command(const comms_packet_view_t *packet)
{
    ESP_LOGI(TAG,
             "TELE id=0x%02x payload_len=%u aguardando fontes de telemetria",
             packet->message_id,
             packet->payload_len);
}

static int handle_command_write(const uint8_t *data, uint16_t len)
{
    comms_packet_view_t packet = {0};

    if (!is_authenticated) {
        ESP_LOGW(TAG, "BLE write comando rejeitado: sem autenticacao len=%u", (unsigned int)len);
        return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
    }

    if (!parse_packet(data, len, &packet)) {
        ESP_LOGW(TAG, "Pacote BLE invalido len=%u", len);
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    pause_regular_telemetry(300);

    ++command_write_count;
    ESP_LOGI(TAG,
             "BLE command #%lu class=%s(0x%02x) id=0x%02x payload=%u total_len=%u",
             (unsigned long)command_write_count,
             command_class_name(packet.command_class),
             packet.command_class,
             packet.message_id,
             packet.payload_len,
             (unsigned int)len);

    switch (packet.command_class) {
    case COMMS_CMD_CLASS_SAVE:
        handle_save_command(&packet);
        break;
    case COMMS_CMD_CLASS_READ:
        handle_read_command(&packet);
        break;
    case COMMS_CMD_CLASS_SEND:
        handle_send_command(&packet);
        break;
    case COMMS_CMD_CLASS_TELE:
        handle_tele_command(&packet);
        break;
    default:
        return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    }

    return 0;
}

static int auth_access_cb(uint16_t conn_handle,
                          uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt,
                          void *arg)
{
    uint8_t data[COMMS_MAX_PAYLOAD_LEN] = {0};
    uint16_t len = 0;
    int rc = 0;

    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }

    rc = copy_mbuf_payload(ctxt->om, data, sizeof(data), &len);
    if (rc != 0) {
        return rc;
    }

    is_authenticated = (len == strlen(COMMS_AUTH_TOKEN)) &&
                       (memcmp(data, COMMS_AUTH_TOKEN, len) == 0);
    if (is_authenticated) {
        auth_deadline_us = 0;
        pause_regular_telemetry(500);
    }

    ESP_LOGI(TAG, "Autenticacao BLE %s", is_authenticated ? "aceita" : "negada");

    return is_authenticated ? 0 : BLE_ATT_ERR_INSUFFICIENT_AUTHOR;
}

static int command_access_cb(uint16_t conn_handle,
                             uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    uint8_t data[COMMS_MAX_PAYLOAD_LEN] = {0};
    uint16_t len = 0;
    int rc = 0;

    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }

    rc = copy_mbuf_payload(ctxt->om, data, sizeof(data), &len);
    if (rc != 0) {
        return rc;
    }

    return handle_command_write(data, len);
}

static int telemetry_access_cb(uint16_t conn_handle,
                               uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt,
                               void *arg)
{
    uint8_t packet[4 + sizeof(comms_imu_telemetry_payload_t)] = {0};
    uint16_t packet_len = 0;
    static const uint8_t empty_telemetry[] = {
        COMMS_PROTOCOL_VERSION,
        COMMS_CMD_CLASS_TELE,
        COMMS_TELE_STATUS,
        1,
        COMMS_ERROR_OK,
    };

    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        return BLE_ATT_ERR_READ_NOT_PERMITTED;
    }

    if (build_imu_telemetry_packet(packet, sizeof(packet), &packet_len)) {
        return os_mbuf_append(ctxt->om, packet, packet_len) == 0
                   ? 0
                   : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    return os_mbuf_append(ctxt->om, empty_telemetry, sizeof(empty_telemetry)) == 0
               ? 0
               : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static const struct ble_gatt_svc_def gatt_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &auth_uuid.u,
                .access_cb = auth_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = &command_uuid.u,
                .access_cb = command_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE,
            },
            {
                .uuid = &telemetry_uuid.u,
                .access_cb = telemetry_access_cb,
                .val_handle = &telemetry_value_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {0},
        },
    },
    {0},
};

static void start_advertising(void)
{
    struct ble_hs_adv_fields fields = {0};
    struct ble_hs_adv_fields response_fields = {0};
    struct ble_gap_adv_params adv_params = {0};
    int rc = 0;

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = &service_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields rc=%d", rc);
        return;
    }

    response_fields.name = (uint8_t *)COMMS_DEVICE_NAME;
    response_fields.name_len = strlen(COMMS_DEVICE_NAME);
    response_fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&response_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields rc=%d", rc);
        return;
    }

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, comms_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "BLE anunciando nome=%s service=%s", COMMS_DEVICE_NAME, COMMS_SERVICE_UUID_STR);
    }
}

static int comms_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "BLE GAP event=%s(%d)", gap_event_name(event->type), event->type);

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        has_active_connection = event->connect.status == 0;
        is_authenticated = false;
        telemetry_subscribed = false;
        if (has_active_connection) {
            active_conn_handle = event->connect.conn_handle;
            active_att_mtu = ble_att_mtu(active_conn_handle);
            if (active_att_mtu < COMMS_BLE_DEFAULT_ATT_MTU) {
                active_att_mtu = COMMS_BLE_DEFAULT_ATT_MTU;
            }
            auth_deadline_us = esp_timer_get_time() + (10 * 1000 * 1000);
            notify_ok_count = 0;
            notify_fail_count = 0;
            command_write_count = 0;
            regular_telemetry_seq = 0;
            telemetry_tick_seq = 0;
            notify_in_flight = false;
            telemetry_tx_backoff_until_us = 0;
            if (tx_queue != NULL) {
                xQueueReset(tx_queue);
            }
            safety_set_ble_connected(true);
            ESP_LOGI(TAG,
                     "BLE conectado conn_handle=%u status=%d mtu=%u",
                     (unsigned int)active_conn_handle,
                     event->connect.status,
                     (unsigned int)active_att_mtu);
            const struct ble_gap_upd_params params = {
                .itvl_min = COMMS_BLE_CONN_ITVL_MIN,
                .itvl_max = COMMS_BLE_CONN_ITVL_MAX,
                .latency = COMMS_BLE_CONN_LATENCY,
                .supervision_timeout = COMMS_BLE_CONN_TIMEOUT,
            };
            int rc = ble_gap_update_params(active_conn_handle, &params);
            if (rc != 0) {
                ESP_LOGW(TAG, "BLE conn params update falhou rc=%d", rc);
            }
        } else {
            ESP_LOGW(TAG, "Falha de conexao BLE status=%d", event->connect.status);
            start_advertising();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG,
                 "BLE desconectado reason=%d conn_handle=%u ok=%lu fail=%lu commands=%lu pending_map=%d received=%u/%u",
                 event->disconnect.reason,
                 (unsigned int)active_conn_handle,
                 (unsigned long)notify_ok_count,
                 (unsigned long)notify_fail_count,
                 (unsigned long)command_write_count,
                 pending_map_active,
                 (unsigned int)pending_map_received_points,
                 (unsigned int)pending_map_total_points);
        has_active_connection = false;
        active_conn_handle = 0;
        active_att_mtu = COMMS_BLE_DEFAULT_ATT_MTU;
        is_authenticated = false;
        telemetry_subscribed = false;
        auth_deadline_us = 0;
        notify_in_flight = false;
        telemetry_tx_backoff_until_us = 0;
        if (tx_queue != NULL) {
            xQueueReset(tx_queue);
        }
        safety_set_ble_connected(false);
        if (!safety_motors_allowed()) {
            control_emergency_stop();
        }
        start_advertising();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == telemetry_value_handle) {
            telemetry_subscribed = event->subscribe.cur_notify != 0;
            ESP_LOGI(TAG,
                     "Notify telemetria %s attr=%u prev_notify=%d cur_notify=%d reason=%d",
                     telemetry_subscribed ? "on" : "off",
                     (unsigned int)event->subscribe.attr_handle,
                     event->subscribe.prev_notify,
                     event->subscribe.cur_notify,
                     event->subscribe.reason);
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        if (event->mtu.conn_handle == active_conn_handle) {
            active_att_mtu = ble_att_mtu(active_conn_handle);
            if (active_att_mtu < event->mtu.value) {
                active_att_mtu = event->mtu.value;
            }
        }
        ESP_LOGI(TAG,
                 "BLE MTU conn=%u channel=%d value=%u active=%u",
                 (unsigned int)event->mtu.conn_handle,
                 event->mtu.channel_id,
                 (unsigned int)event->mtu.value,
                 (unsigned int)active_att_mtu);
        return 0;

    case BLE_GAP_EVENT_NOTIFY_TX:
        if (event->notify_tx.conn_handle == active_conn_handle &&
            event->notify_tx.attr_handle == telemetry_value_handle) {
            notify_in_flight = false;
            if (event->notify_tx.status == 0) {
                telemetry_tx_backoff_until_us = 0;
            } else {
                telemetry_tx_backoff_until_us =
                    esp_timer_get_time() + ((int64_t)COMMS_BLE_NOTIFY_TX_BACKOFF_MS * 1000);
            }
            if (tx_task_handle != NULL) {
                xTaskNotifyGive(tx_task_handle);
            }
        }
        if (event->notify_tx.status != 0) {
            ESP_LOGW(TAG,
                     "BLE notify_tx status=%d attr=%u indication=%d",
                     event->notify_tx.status,
                     (unsigned int)event->notify_tx.attr_handle,
                     event->notify_tx.indication);
        }
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGI(TAG,
                 "BLE conn update status=%d conn=%u",
                 event->conn_update.status,
                 (unsigned int)event->conn_update.conn_handle);
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "BLE advertising complete reason=%d; reiniciando", event->adv_complete.reason);
        start_advertising();
        return 0;

    default:
        return 0;
    }
}

static void on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE reset reason=%d", reason);
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    assert(rc == 0);

    start_advertising();
}

static void host_task(void *param)
{
    (void)param;

    ESP_LOGI(TAG, "Host BLE iniciado");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t comms_ble_init(void)
{
    esp_log_level_set("NimBLE", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    int rc = nimble_port_init();

    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init falhou rc=%d", rc);
        return ESP_FAIL;
    }

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    rc = ble_gatts_count_cfg(gatt_services);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg rc=%d", rc);
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(gatt_services);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs rc=%d", rc);
        return ESP_FAIL;
    }

    rc = ble_svc_gap_device_name_set(COMMS_DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_svc_gap_device_name_set rc=%d", rc);
        return ESP_FAIL;
    }

    ble_store_config_init();
    nimble_port_freertos_init(host_task);

    if (tx_queue == NULL) {
        tx_queue = xQueueCreate(COMMS_BLE_TX_QUEUE_DEPTH, sizeof(comms_ble_tx_item_t));
        if (tx_queue == NULL) {
            ESP_LOGE(TAG, "Falha ao criar fila TX BLE");
            return ESP_ERR_NO_MEM;
        }
    }

    if (tx_task_handle == NULL) {
        BaseType_t task_created = xTaskCreatePinnedToCore(tx_task,
                                                          "ble_tx",
                                                          4096,
                                                          NULL,
                                                          COMMS_BLE_TX_TASK_PRIORITY,
                                                          &tx_task_handle,
                                                          COMMS_BLE_TASK_CORE_ID);
        if (task_created != pdPASS) {
            ESP_LOGE(TAG, "Falha ao criar task TX BLE");
            return ESP_FAIL;
        }
    }

    if (telemetry_task_handle == NULL) {
        BaseType_t task_created = xTaskCreatePinnedToCore(telemetry_task,
                                                          "ble_telemetry",
                                                          4096,
                                                          NULL,
                                                          COMMS_BLE_TELEMETRY_TASK_PRIORITY,
                                                          &telemetry_task_handle,
                                                          COMMS_BLE_TASK_CORE_ID);
        if (task_created != pdPASS) {
            ESP_LOGE(TAG, "Falha ao criar task de telemetria BLE");
            return ESP_FAIL;
        }
    }

    if (map_save_task_handle == NULL) {
        BaseType_t task_created = xTaskCreatePinnedToCore(map_save_task,
                                                          "ble_map_save",
                                                          4096,
                                                          NULL,
                                                          COMMS_BLE_MAP_SAVE_TASK_PRIORITY,
                                                          &map_save_task_handle,
                                                          COMMS_BLE_TASK_CORE_ID);
        if (task_created != pdPASS) {
            ESP_LOGE(TAG, "Falha ao criar task de save de mapas BLE");
            return ESP_FAIL;
        }
    }

    if (telemetry_timer_handle == NULL) {
        const esp_timer_create_args_t timer_args = {
            .callback = telemetry_timer_cb,
            .name = "ble_sensor_30hz",
        };
        esp_err_t timer_ret = esp_timer_create(&timer_args, &telemetry_timer_handle);
        if (timer_ret != ESP_OK) {
            ESP_LOGE(TAG, "Falha ao criar timer de telemetria BLE ret=%s", esp_err_to_name(timer_ret));
            return timer_ret;
        }

        timer_ret = esp_timer_start_periodic(telemetry_timer_handle, COMMS_BLE_SENSOR_TELEMETRY_PERIOD_US);
        if (timer_ret != ESP_OK) {
            ESP_LOGE(TAG, "Falha ao iniciar timer de telemetria BLE ret=%s", esp_err_to_name(timer_ret));
            return timer_ret;
        }
    }

    ESP_LOGI(TAG,
             "BLE pronto device=%s service=%s auth=%s sensor=%.1fHz linha_fast=%.1fHz linha_full=%.1fHz tele=%.1fHz core=%d txq=%d",
             COMMS_DEVICE_NAME,
             COMMS_SERVICE_UUID_STR,
             COMMS_AUTH_UUID_STR,
             1000000.0f / (float)COMMS_BLE_SENSOR_TELEMETRY_PERIOD_US,
             1000000.0f / (float)COMMS_BLE_SENSOR_TELEMETRY_PERIOD_US,
             1000000.0f / ((float)COMMS_BLE_SENSOR_TELEMETRY_PERIOD_US * (float)COMMS_BLE_FULL_LINE_TELEMETRY_DIVIDER),
             1000000.0f / ((float)COMMS_BLE_SENSOR_TELEMETRY_PERIOD_US * (float)COMMS_BLE_REGULAR_TELEMETRY_DIVIDER),
             COMMS_BLE_TASK_CORE_ID,
             COMMS_BLE_TX_QUEUE_DEPTH);

    return ESP_OK;
}
