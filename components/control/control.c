#include "control.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "battery_level.h"
#include "battery_level_config.h"
#include "control_config.h"
#include "encoder.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "memory_config.h"
#include "memory_maps.h"
#include "nvs.h"
#include "imu.h"
#include "imu_config.h"
#include "line_sensor.h"
#include "odometry.h"
#include "odometry_config.h"
#include "rgb_led.h"
#include "safety.h"

#define CONTROL_NVS_KEY_KP_MILLI "ctrl_kp"
#define CONTROL_NVS_KEY_KI_MILLI "ctrl_ki"
#define CONTROL_NVS_KEY_KD_MILLI "ctrl_kd"
#define CONTROL_NVS_KEY_LIMIT "ctrl_lim"
#define CONTROL_NVS_KEY_AUX "ctrl_aux"
#define CONTROL_NVS_KEY_SPEED_PROFILE "ctrl_spd_prof"
#define CONTROL_NVS_KEY_SPEED_PROFILE_ENABLED "ctrl_spd_en"
#define CONTROL_NVS_KEY_BATTERY_COMPENSATION "ctrl_bat_comp"
#define CONTROL_NVS_KEY_AUTO_TRACK_CONFIG "ctrl_auto_cfg"
#define CONTROL_AUTO_TRACK_CONFIG_LEGACY_LINE_LOSS_OFFSET (sizeof(uint8_t) + sizeof(uint8_t) + sizeof(float))
#define CONTROL_AUTO_TRACK_CONFIG_LEGACY_SIZE \
    (CONTROL_AUTO_TRACK_CONFIG_LEGACY_LINE_LOSS_OFFSET + sizeof(uint8_t))
#define CONTROL_AUTO_TRACK_CONFIG_LEGACY_WITH_LINE_ERROR_SIZE \
    (CONTROL_AUTO_TRACK_CONFIG_LEGACY_SIZE + sizeof(float) + sizeof(float))

#if CONTROL_TRACK_ODOMETRY_TASK_PERIOD_US != ODOMETRY_UPDATE_PERIOD_US
#error "CONTROL_TRACK_ODOMETRY_TASK_PERIOD_US deve acompanhar ODOMETRY_UPDATE_PERIOD_US."
#endif

#if CONTROL_TRACK_ODOMETRY_TASK_PERIOD_US != ENCODER_SAMPLE_PERIOD_US
#error "CONTROL_TRACK_ODOMETRY_TASK_PERIOD_US deve acompanhar ENCODER_SAMPLE_PERIOD_US."
#endif

#if CONTROL_TASK_CORE_ID == LINE_SENSOR_TASK_CORE_ID && CONTROL_TASK_PRIORITY <= LINE_SENSOR_TASK_PRIORITY
#error "CONTROL_TASK_PRIORITY deve ser maior que LINE_SENSOR_TASK_PRIORITY no mesmo nucleo."
#endif

#if CONTROL_TRACK_ODOMETRY_TASK_CORE_ID == LINE_SENSOR_TASK_CORE_ID && CONTROL_TRACK_ODOMETRY_TASK_PRIORITY <= LINE_SENSOR_TASK_PRIORITY
#error "CONTROL_TRACK_ODOMETRY_TASK_PRIORITY deve ser maior que LINE_SENSOR_TASK_PRIORITY no mesmo nucleo."
#endif

#if CONTROL_RACE_PLAN_TASK_CORE_ID == LINE_SENSOR_TASK_CORE_ID && CONTROL_RACE_PLAN_TASK_PRIORITY <= LINE_SENSOR_TASK_PRIORITY
#error "CONTROL_RACE_PLAN_TASK_PRIORITY deve ser maior que LINE_SENSOR_TASK_PRIORITY no mesmo nucleo."
#endif

#if IMU_TASK_CORE_ID == LINE_SENSOR_TASK_CORE_ID && IMU_TASK_PRIORITY <= LINE_SENSOR_TASK_PRIORITY
#error "IMU_TASK_PRIORITY deve ser maior que LINE_SENSOR_TASK_PRIORITY no mesmo nucleo."
#endif

#if IMU_TASK_CORE_ID == CONTROL_TASK_CORE_ID && IMU_TASK_PRIORITY >= CONTROL_TASK_PRIORITY
#error "IMU_TASK_PRIORITY deve ficar abaixo de CONTROL_TASK_PRIORITY no nucleo de controle."
#endif

#define CONTROL_LINE_CENTER_POSITION 3500.0f
#define CONTROL_LINE_POSITION_SCALE 3500.0f
#define CONTROL_LINE_STEER_SIGN 1.0f
#define CONTROL_FIRST_MAP_TARGET_INDEX 1
#define CONTROL_BATTERY_COMPENSATION_MIN_V 1.0f

static const char *TAG = "control";
static const float CONTROL_PI_F = 3.14159265f;

static TaskHandle_t control_task_handle;
static TaskHandle_t race_plan_task_handle;
static TaskHandle_t track_odometry_task_handle;
static esp_timer_handle_t control_timer_handle;
static esp_timer_handle_t race_plan_timer_handle;
static esp_timer_handle_t track_odometry_timer_handle;
static portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;
static control_navigation_state_t nav_state;
static control_speed_profile_point_t speed_profile[CONTROL_SPEED_PROFILE_POINTS];
static control_auto_track_config_t auto_track_config;
static control_race_plan_segment_t race_plan_segments[CONTROL_RACE_PLAN_MAX_SEGMENTS];
static uint8_t race_plan_segment_count;
static bool race_plan_enabled;
static bool race_plan_runtime_enabled;
static int8_t race_plan_direction = 1;
static int64_t race_plan_start_us;
static float race_plan_track_distance_m;
static memory_map_point_t active_map[MEMORY_MAP_MAX_POINTS];
static float active_map_distance_m[MEMORY_MAP_MAX_POINTS];
static int32_t map_start_left_count;
static int32_t map_start_right_count;
static bool map_encoder_reference_valid;
static float manual_kp;
static float manual_ki;
static float manual_kd;
static float error_integral_rad_s;
static float previous_error_rad;
static bool have_previous_error;
static bool initialized;
static float speed_avg_samples[CONTROL_SPEED_AVG_WINDOW_MS];
static uint16_t speed_avg_index;
static uint16_t speed_avg_count;
static float speed_avg_sum;
static float speed_max_mps;

static float wrap_pi(float angle);
static void race_plan_task(void *arg);
static void track_odometry_task(void *arg);

typedef struct {
    uint16_t target_index;
    float target_x_m;
    float target_y_m;
    float map_x_m;
    float map_y_m;
    float map_heading_rad;
    float distance_to_target_m;
    float progress_m;
    bool complete;
} control_map_progress_t;

static int32_t gain_to_milli(float value)
{
    return (int32_t)lroundf(value * 1000.0f);
}

static float milli_to_gain(int32_t value)
{
    return (float)value / 1000.0f;
}

static void set_default_speed_profile(void)
{
    const uint8_t speeds[CONTROL_SPEED_PROFILE_POINTS] = {0, 25, 50, 75, 100};
    for (int i = 0; i < CONTROL_SPEED_PROFILE_POINTS; ++i) {
        speed_profile[i].speed_percent = speeds[i];
        speed_profile[i].kp = CONTROL_HEADING_KP_PERCENT_PER_RAD;
        speed_profile[i].ki = CONTROL_HEADING_KI_PERCENT_PER_RAD_S;
        speed_profile[i].kd = CONTROL_HEADING_KD_PERCENT_S_PER_RAD;
        speed_profile[i].aux_percent = 0;
    }
}

static void set_default_auto_track_config(void)
{
    auto_track_config.line_loss_odometry_enabled = 1;
}

static bool speed_profile_is_valid(const control_speed_profile_point_t *points, size_t count)
{
    if (points == NULL || count != CONTROL_SPEED_PROFILE_POINTS) {
        return false;
    }
    uint8_t previous_speed = 0;
    for (size_t i = 0; i < count; ++i) {
        if (points[i].speed_percent > 100 || points[i].aux_percent > 100 ||
            !isfinite(points[i].kp) || !isfinite(points[i].ki) || !isfinite(points[i].kd) ||
            points[i].kp < 0.0f || points[i].ki < 0.0f || points[i].kd < 0.0f ||
            points[i].kp > CONTROL_MAX_PID_GAIN || points[i].ki > CONTROL_MAX_PID_GAIN ||
            points[i].kd > CONTROL_MAX_PID_GAIN) {
            return false;
        }
        if (i > 0 && points[i].speed_percent <= previous_speed) {
            return false;
        }
        previous_speed = points[i].speed_percent;
    }
    return true;
}

static bool race_plan_is_valid(const control_race_plan_segment_t *segments, size_t count)
{
    if (count == 0) {
        return true;
    }
    if (segments == NULL || count > CONTROL_RACE_PLAN_MAX_SEGMENTS) {
        return false;
    }

    uint16_t previous_end = 0;
    for (size_t i = 0; i < count; ++i) {
        const control_race_plan_segment_t *segment = &segments[i];
        if (segment->start_index > segment->end_index ||
            segment->type > CONTROL_RACE_SEGMENT_INTERSECTION ||
            segment->speed_percent > 100 ||
            segment->max_speed_percent > 100 ||
            segment->aux_percent > 100 ||
            !isfinite(segment->kp) || !isfinite(segment->ki) || !isfinite(segment->kd) ||
            segment->kp < 0.0f || segment->ki < 0.0f || segment->kd < 0.0f ||
            segment->kp > CONTROL_MAX_PID_GAIN || segment->ki > CONTROL_MAX_PID_GAIN ||
            segment->kd > CONTROL_MAX_PID_GAIN) {
            return false;
        }
        if (i > 0 && segment->start_index <= previous_end) {
            return false;
        }
        previous_end = segment->end_index;
    }
    return true;
}

static uint8_t abs_speed_percent(int8_t speed_percent)
{
    int speed = abs((int)speed_percent);
    if (speed > 100) {
        speed = 100;
    }
    return (uint8_t)speed;
}

static int signed_speed_percent(int8_t requested_speed, uint8_t magnitude)
{
    return requested_speed < 0 ? -(int)magnitude : (int)magnitude;
}

static float encoder_counts_to_meters(int32_t counts)
{
    return ((float)counts * ODOMETRY_WHEEL_CIRCUMFERENCE_M) /
           ODOMETRY_ENCODER_COUNTS_PER_WHEEL_REV;
}

static void build_active_map_distances(uint16_t total_points)
{
    if (total_points == 0) {
        return;
    }

    active_map_distance_m[0] = 0.0f;
    for (uint16_t i = 1; i < total_points && i < MEMORY_MAP_MAX_POINTS; ++i) {
        const float dx = active_map[i].x_m - active_map[i - 1].x_m;
        const float dy = active_map[i].y_m - active_map[i - 1].y_m;
        const float segment_m = sqrtf((dx * dx) + (dy * dy));
        active_map_distance_m[i] = active_map_distance_m[i - 1] + segment_m;
    }
}

static void reset_map_encoder_reference(void)
{
    encoder_state_t encoder = {0};
    if (encoder_get_state(&encoder)) {
        map_start_left_count = encoder.counts[ENCODER_LEFT];
        map_start_right_count = encoder.counts[ENCODER_RIGHT];
        map_encoder_reference_valid = true;
    } else {
        map_start_left_count = 0;
        map_start_right_count = 0;
        map_encoder_reference_valid = false;
    }
}

static float map_encoder_progress_m(int8_t speed_percent)
{
    if (!map_encoder_reference_valid) {
        reset_map_encoder_reference();
        if (!map_encoder_reference_valid) {
            return 0.0f;
        }
    }

    encoder_state_t encoder = {0};
    if (!encoder_get_state(&encoder)) {
        return 0.0f;
    }

    float left_m = encoder_counts_to_meters(encoder.counts[ENCODER_LEFT] - map_start_left_count);
    float right_m = encoder_counts_to_meters(encoder.counts[ENCODER_RIGHT] - map_start_right_count);
    float progress_m = (left_m + right_m) * 0.5f;
    if (speed_percent < 0) {
        progress_m = -progress_m;
    }
    if (progress_m < 0.0f) {
        progress_m = 0.0f;
    }
    return progress_m;
}

static bool map_progress_from_encoder(const control_navigation_state_t *local, control_map_progress_t *out_progress)
{
    if (local == NULL || out_progress == NULL || local->point_count < 2) {
        return false;
    }

    const uint16_t last_index = (uint16_t)(local->point_count - 1);
    const float total_distance_m = active_map_distance_m[last_index];
    if (!isfinite(total_distance_m) || total_distance_m <= 0.0f) {
        return false;
    }

    float progress_m = map_encoder_progress_m(local->speed_percent);
    if (progress_m > total_distance_m) {
        progress_m = total_distance_m;
    }

    uint16_t target_index = 1;
    while (target_index < last_index &&
           active_map_distance_m[target_index] <= progress_m + CONTROL_TARGET_REACHED_DISTANCE_M) {
        ++target_index;
    }

    uint16_t segment_end_index = 1;
    while (segment_end_index < last_index && active_map_distance_m[segment_end_index] < progress_m) {
        ++segment_end_index;
    }
    const uint16_t segment_start_index = segment_end_index > 0 ? (uint16_t)(segment_end_index - 1) : 0;
    const float start_distance = active_map_distance_m[segment_start_index];
    const float end_distance = active_map_distance_m[segment_end_index];
    const float segment_distance = end_distance - start_distance;
    float t = segment_distance > 0.0001f ? (progress_m - start_distance) / segment_distance : 0.0f;
    if (t < 0.0f) {
        t = 0.0f;
    } else if (t > 1.0f) {
        t = 1.0f;
    }

    const memory_map_point_t start = active_map[segment_start_index];
    const memory_map_point_t end = active_map[segment_end_index];
    const float dx = end.x_m - start.x_m;
    const float dy = end.y_m - start.y_m;

    out_progress->target_index = target_index;
    out_progress->target_x_m = active_map[target_index].x_m;
    out_progress->target_y_m = active_map[target_index].y_m;
    out_progress->map_x_m = start.x_m + (dx * t);
    out_progress->map_y_m = start.y_m + (dy * t);
    out_progress->map_heading_rad = atan2f(dy, dx);
    out_progress->distance_to_target_m = active_map_distance_m[target_index] - progress_m;
    if (out_progress->distance_to_target_m < 0.0f) {
        out_progress->distance_to_target_m = 0.0f;
    }
    out_progress->progress_m = progress_m;
    out_progress->complete = progress_m >= total_distance_m;
    return true;
}

static esp_err_t load_active_map(uint8_t map_slot, uint16_t *out_total_points)
{
    if (out_total_points == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t total_points = 0;
    uint16_t offset = 0;
    while (true) {
        uint8_t count = 0;
        esp_err_t ret = memory_maps_load_chunk(map_slot,
                                               offset,
                                               &active_map[offset],
                                               MEMORY_MAP_CHUNK_MAX_POINTS,
                                               &total_points,
                                               &count);
        if (ret != ESP_OK) {
            return ret;
        }
        offset += count;
        if (offset >= total_points || count == 0) {
            break;
        }
    }

    if (total_points < 2 || total_points > MEMORY_MAP_MAX_POINTS) {
        return ESP_ERR_INVALID_SIZE;
    }

    *out_total_points = total_points;
    build_active_map_distances(total_points);
    return ESP_OK;
}

typedef struct {
    control_speed_profile_point_t profile;
    int drive_speed_percent;
    uint8_t selected_speed_percent;
    uint8_t motor_limit_percent;
    bool race_segment_active;
    control_race_plan_segment_t race_segment;
    bool stop;
} control_drive_selection_t;

static void race_plan_led_color(control_race_segment_type_t type, uint8_t *red, uint8_t *green, uint8_t *blue)
{
    switch (type) {
    case CONTROL_RACE_SEGMENT_INTERSECTION:
        *red = 86;
        *green = 204;
        *blue = 242;
        break;
    case CONTROL_RACE_SEGMENT_CURVE:
        *red = 242;
        *green = 153;
        *blue = 74;
        break;
    case CONTROL_RACE_SEGMENT_STOP:
        *red = 235;
        *green = 87;
        *blue = 87;
        break;
    case CONTROL_RACE_SEGMENT_NORMAL:
    default:
        *red = 39;
        *green = 174;
        *blue = 96;
        break;
    }
}

static void update_race_plan_led(control_race_segment_type_t type)
{
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
    race_plan_led_color(type, &red, &green, &blue);
    rgb_led_set_race_plan_color(red, green, blue);
}

static bool race_plan_segment_for_target_index(uint16_t target_index,
                                               control_race_plan_segment_t *out_segment,
                                               uint8_t *out_index)
{
    if (out_segment == NULL) {
        return false;
    }

    bool found = false;
    portENTER_CRITICAL(&state_mux);
    if (race_plan_enabled) {
        for (uint8_t i = 0; i < race_plan_segment_count; ++i) {
            const control_race_plan_segment_t segment = race_plan_segments[i];
            if (target_index >= segment.start_index && target_index <= segment.end_index) {
                *out_segment = segment;
                if (out_index != NULL) {
                    *out_index = i;
                }
                found = true;
                break;
            }
        }
    }
    portEXIT_CRITICAL(&state_mux);
    return found;
}

static bool race_plan_segment_for_target(uint16_t target_index, control_race_plan_segment_t *out_segment)
{
    return race_plan_segment_for_target_index(target_index, out_segment, NULL);
}

static bool race_plan_segment_is_drive_profile(const control_race_plan_segment_t *segment)
{
    return segment != NULL &&
           segment->type != CONTROL_RACE_SEGMENT_STOP &&
           segment->type != CONTROL_RACE_SEGMENT_INTERSECTION;
}

static bool race_plan_neighbor_profile(uint8_t segment_index, int direction, control_race_plan_segment_t *out_segment)
{
    if (out_segment == NULL || direction == 0) {
        return false;
    }

    bool found = false;
    portENTER_CRITICAL(&state_mux);
    int index = (int)segment_index + direction;
    while (index >= 0 && index < (int)race_plan_segment_count) {
        const control_race_plan_segment_t candidate = race_plan_segments[index];
        if (race_plan_segment_is_drive_profile(&candidate)) {
            *out_segment = candidate;
            found = true;
            break;
        }
        index += direction;
    }
    portEXIT_CRITICAL(&state_mux);
    return found;
}

static uint8_t lerp_u8(uint8_t from, uint8_t to, float t)
{
    const float value = (float)from + (((float)to - (float)from) * t);
    if (value <= 0.0f) {
        return 0;
    }
    if (value >= 100.0f) {
        return 100;
    }
    return (uint8_t)lroundf(value);
}

static float lerp_float(float from, float to, float t)
{
    return from + ((to - from) * t);
}

static control_race_plan_segment_t race_plan_effective_segment(const control_race_plan_segment_t *segment,
                                                               uint8_t segment_index,
                                                               float progress_m)
{
    if (segment == NULL) {
        return (control_race_plan_segment_t){0};
    }
    control_race_plan_segment_t effective = *segment;
    if (segment->type != CONTROL_RACE_SEGMENT_INTERSECTION ||
        segment->end_index >= MEMORY_MAP_MAX_POINTS) {
        return effective;
    }

    control_race_plan_segment_t previous = *segment;
    control_race_plan_segment_t next = *segment;
    const bool has_previous = race_plan_neighbor_profile(segment_index, -1, &previous);
    const bool has_next = race_plan_neighbor_profile(segment_index, 1, &next);
    if (!has_previous && !has_next) {
        return effective;
    }
    if (!has_previous) {
        previous = next;
    }
    if (!has_next) {
        next = previous;
    }

    const uint16_t start_index = segment->start_index > 0 ? (uint16_t)(segment->start_index - 1U) : 0;
    const uint16_t end_index = segment->end_index;
    const float start_m = active_map_distance_m[start_index];
    const float end_m = active_map_distance_m[end_index];
    const float span_m = end_m - start_m;
    float t = span_m > 0.0001f ? (progress_m - start_m) / span_m : 0.0f;
    if (t < 0.0f) {
        t = 0.0f;
    } else if (t > 1.0f) {
        t = 1.0f;
    }

    effective.speed_percent = lerp_u8(previous.speed_percent, next.speed_percent, t);
    effective.max_speed_percent = lerp_u8(previous.max_speed_percent, next.max_speed_percent, t);
    effective.aux_percent = lerp_u8(previous.aux_percent, next.aux_percent, t);
    effective.kp = lerp_float(previous.kp, next.kp, t);
    effective.ki = lerp_float(previous.ki, next.ki, t);
    effective.kd = lerp_float(previous.kd, next.kd, t);
    return effective;
}

static control_drive_selection_t select_drive_for_target(const control_navigation_state_t *local,
                                                         bool use_race_plan,
                                                         uint16_t target_index,
                                                         float manual_kp_value,
                                                         float manual_ki_value,
                                                         float manual_kd_value)
{
    control_drive_selection_t selection = {0};
    uint8_t selected_speed_percent = abs_speed_percent(local->speed_percent);

    selection.profile.speed_percent = selected_speed_percent;
    selection.profile.kp = manual_kp_value;
    selection.profile.ki = manual_ki_value;
    selection.profile.kd = manual_kd_value;
    selection.profile.aux_percent = local->aux_percent;
    selection.motor_limit_percent = local->motor_limit_percent;

    control_race_plan_segment_t segment = {0};
    if (use_race_plan && race_plan_segment_for_target(target_index, &segment)) {
        selection.race_segment_active = true;
        selection.race_segment = segment;
        selected_speed_percent = segment.speed_percent;
        selection.profile.speed_percent = selected_speed_percent;
        selection.profile.aux_percent = segment.aux_percent;
        selection.profile.kp = segment.kp;
        selection.profile.ki = segment.ki;
        selection.profile.kd = segment.kd;
        selection.motor_limit_percent = segment.max_speed_percent;
        selection.stop = segment.type == CONTROL_RACE_SEGMENT_STOP;
        update_race_plan_led((control_race_segment_type_t)segment.type);
    }

    selection.selected_speed_percent = selected_speed_percent;
    selection.drive_speed_percent = signed_speed_percent(local->speed_percent, selected_speed_percent);
    return selection;
}

static control_drive_selection_t select_line_drive(const control_navigation_state_t *local,
                                                   float manual_kp_value,
                                                   float manual_ki_value,
                                                   float manual_kd_value)
{
    control_drive_selection_t selection = {0};
    const uint8_t selected_speed_percent = abs_speed_percent(local->speed_percent);

    selection.profile.speed_percent = selected_speed_percent;
    selection.profile.kp = manual_kp_value;
    selection.profile.ki = manual_ki_value;
    selection.profile.kd = manual_kd_value;
    selection.profile.aux_percent = local->aux_percent;
    selection.motor_limit_percent = local->motor_limit_percent;

    if (local->race_segment_active) {
        selection.profile.kp = local->kp;
        selection.profile.ki = local->ki;
        selection.profile.kd = local->kd;
    }

    selection.selected_speed_percent = selected_speed_percent;
    selection.drive_speed_percent = signed_speed_percent(local->speed_percent, selected_speed_percent);
    return selection;
}

static void update_active_race_segment_locked(const control_drive_selection_t *drive)
{
    if (drive != NULL && drive->race_segment_active) {
        nav_state.race_segment_active = true;
        nav_state.race_segment_type = drive->race_segment.type;
        nav_state.race_segment_start_index = drive->race_segment.start_index;
        nav_state.race_segment_end_index = drive->race_segment.end_index;
        nav_state.race_segment_speed_percent = drive->race_segment.speed_percent;
        nav_state.race_segment_max_speed_percent = drive->race_segment.max_speed_percent;
        nav_state.race_segment_aux_percent = drive->race_segment.aux_percent;
    } else {
        nav_state.race_segment_active = false;
        nav_state.race_segment_type = CONTROL_RACE_SEGMENT_NORMAL;
        nav_state.race_segment_start_index = 0;
        nav_state.race_segment_end_index = 0;
        nav_state.race_segment_speed_percent = 0;
        nav_state.race_segment_max_speed_percent = 0;
        nav_state.race_segment_aux_percent = 0;
    }
}

static void load_settings_from_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        return;
    }

    int32_t stored = 0;
    uint8_t limit = CONTROL_DEFAULT_MOTOR_LIMIT_PERCENT;
    if (nvs_get_i32(handle, CONTROL_NVS_KEY_KP_MILLI, &stored) == ESP_OK && stored >= 0) {
        manual_kp = milli_to_gain(stored);
        nav_state.kp = manual_kp;
    }
    if (nvs_get_i32(handle, CONTROL_NVS_KEY_KI_MILLI, &stored) == ESP_OK && stored >= 0) {
        manual_ki = milli_to_gain(stored);
        nav_state.ki = manual_ki;
    }
    if (nvs_get_i32(handle, CONTROL_NVS_KEY_KD_MILLI, &stored) == ESP_OK && stored >= 0) {
        manual_kd = milli_to_gain(stored);
        nav_state.kd = manual_kd;
    }
    if (nvs_get_u8(handle, CONTROL_NVS_KEY_LIMIT, &limit) == ESP_OK && limit <= 100) {
        nav_state.motor_limit_percent = limit;
    }
    uint8_t aux_percent = nav_state.aux_percent;
    if (nvs_get_u8(handle, CONTROL_NVS_KEY_AUX, &aux_percent) == ESP_OK && aux_percent <= 100) {
        nav_state.aux_percent = aux_percent;
    }
    uint8_t speed_profile_enabled = nav_state.speed_profile_enabled ? 1 : 0;
    if (nvs_get_u8(handle, CONTROL_NVS_KEY_SPEED_PROFILE_ENABLED, &speed_profile_enabled) == ESP_OK) {
        nav_state.speed_profile_enabled = speed_profile_enabled != 0;
    }
    uint8_t battery_compensation_enabled = nav_state.battery_compensation_enabled ? 1 : 0;
    if (nvs_get_u8(handle, CONTROL_NVS_KEY_BATTERY_COMPENSATION, &battery_compensation_enabled) == ESP_OK) {
        nav_state.battery_compensation_enabled = battery_compensation_enabled != 0;
    }
    control_speed_profile_point_t stored_profile[CONTROL_SPEED_PROFILE_POINTS] = {0};
    size_t profile_len = sizeof(stored_profile);
    if (nvs_get_blob(handle, CONTROL_NVS_KEY_SPEED_PROFILE, stored_profile, &profile_len) == ESP_OK &&
        profile_len == sizeof(stored_profile) &&
        speed_profile_is_valid(stored_profile, CONTROL_SPEED_PROFILE_POINTS)) {
        memcpy(speed_profile, stored_profile, sizeof(speed_profile));
    }
    nav_state.speed_profile_enabled = false;
    uint8_t stored_auto_config[CONTROL_AUTO_TRACK_CONFIG_LEGACY_WITH_LINE_ERROR_SIZE] = {0};
    size_t auto_config_len = sizeof(stored_auto_config);
    if (nvs_get_blob(handle, CONTROL_NVS_KEY_AUTO_TRACK_CONFIG, stored_auto_config, &auto_config_len) == ESP_OK) {
        if (auto_config_len == sizeof(control_auto_track_config_t)) {
            auto_track_config.line_loss_odometry_enabled = stored_auto_config[0] ? 1 : 0;
        } else if (auto_config_len == CONTROL_AUTO_TRACK_CONFIG_LEGACY_SIZE ||
                   auto_config_len == CONTROL_AUTO_TRACK_CONFIG_LEGACY_WITH_LINE_ERROR_SIZE) {
            auto_track_config.line_loss_odometry_enabled =
                stored_auto_config[CONTROL_AUTO_TRACK_CONFIG_LEGACY_LINE_LOSS_OFFSET] ? 1 : 0;
        }
    }
    nvs_close(handle);
}

static esp_err_t save_settings_to_nvs(float kp, float ki, float kd, uint8_t limit_percent, uint8_t aux_percent)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = nvs_set_i32(handle, CONTROL_NVS_KEY_KP_MILLI, gain_to_milli(kp));
    if (ret == ESP_OK) {
        ret = nvs_set_i32(handle, CONTROL_NVS_KEY_KI_MILLI, gain_to_milli(ki));
    }
    if (ret == ESP_OK) {
        ret = nvs_set_i32(handle, CONTROL_NVS_KEY_KD_MILLI, gain_to_milli(kd));
    }
    if (ret == ESP_OK) {
        ret = nvs_set_u8(handle, CONTROL_NVS_KEY_LIMIT, limit_percent);
    }
    if (ret == ESP_OK) {
        ret = nvs_set_u8(handle, CONTROL_NVS_KEY_AUX, aux_percent);
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    return ret;
}

static esp_err_t save_speed_profile_enabled_to_nvs(bool enabled)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = nvs_set_u8(handle, CONTROL_NVS_KEY_SPEED_PROFILE_ENABLED, enabled ? 1 : 0);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    return ret;
}

static esp_err_t save_battery_compensation_enabled_to_nvs(bool enabled)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = nvs_set_u8(handle, CONTROL_NVS_KEY_BATTERY_COMPENSATION, enabled ? 1 : 0);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    return ret;
}

static esp_err_t save_auto_track_config_to_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = nvs_set_blob(handle, CONTROL_NVS_KEY_AUTO_TRACK_CONFIG, &auto_track_config, sizeof(auto_track_config));
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    return ret;
}

static float wrap_pi(float angle)
{
    while (angle > CONTROL_PI_F) {
        angle -= 2.0f * CONTROL_PI_F;
    }
    while (angle < -CONTROL_PI_F) {
        angle += 2.0f * CONTROL_PI_F;
    }
    return angle;
}

static float battery_voltage_scale(bool enabled)
{
    if (!enabled) {
        return 1.0f;
    }

    battery_level_state_t battery = {0};
    if (battery_level_get_state(&battery) &&
        battery.valid &&
        isfinite(battery.voltage_v) &&
        battery.voltage_v >= CONTROL_BATTERY_COMPENSATION_MIN_V) {
        return BATTERY_LEVEL_VOLTAGE_MAX_V / battery.voltage_v;
    }

    return 1.0f;
}

static int clamp_motor_percent(float value, uint8_t limit_percent, bool battery_compensation_enabled)
{
    const float voltage_scale = battery_voltage_scale(battery_compensation_enabled);
    float limit = (float)limit_percent * voltage_scale;
    value *= voltage_scale;
    if (limit > 100.0f) {
        limit = 100.0f;
    }
    if (value > limit) {
        return (int)limit;
    }
    if (value < -limit) {
        return (int)-limit;
    }
    return (int)lroundf(value);
}

static int compensate_motor_percent(uint8_t percent, bool battery_compensation_enabled)
{
    return clamp_motor_percent((float)percent, percent, battery_compensation_enabled);
}

static float clamp_abs(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static void update_error_integral(float error, float dt_s)
{
    if (fabsf(error) <= CONTROL_INTEGRAL_RESET_ERROR_RAD) {
        error_integral_rad_s = 0.0f;
        return;
    }

    error_integral_rad_s = clamp_abs(error_integral_rad_s + (error * dt_s), CONTROL_MAX_INTEGRAL_RAD_S);
}

static void reset_speed_stats(void)
{
    memset(speed_avg_samples, 0, sizeof(speed_avg_samples));
    speed_avg_index = 0;
    speed_avg_count = 0;
    speed_avg_sum = 0.0f;
    speed_max_mps = 0.0f;
}

static void update_speed_stats(float linear_mps)
{
    const float speed_mps = fabsf(linear_mps);
    if (speed_avg_count < CONTROL_SPEED_AVG_WINDOW_MS) {
        speed_avg_samples[speed_avg_index] = speed_mps;
        speed_avg_sum += speed_mps;
        ++speed_avg_count;
    } else {
        speed_avg_sum -= speed_avg_samples[speed_avg_index];
        speed_avg_samples[speed_avg_index] = speed_mps;
        speed_avg_sum += speed_mps;
    }
    speed_avg_index = (uint16_t)((speed_avg_index + 1U) % CONTROL_SPEED_AVG_WINDOW_MS);
    if (speed_mps > speed_max_mps) {
        speed_max_mps = speed_mps;
    }

    portENTER_CRITICAL(&state_mux);
    nav_state.average_speed_mps = speed_avg_count > 0 ? speed_avg_sum / (float)speed_avg_count : 0.0f;
    nav_state.max_speed_mps = speed_max_mps;
    portEXIT_CRITICAL(&state_mux);
}

static float finish_race_plan_average_locked(int64_t stop_us)
{
    float average_mps = nav_state.race_plan_average_speed_mps;
    if (race_plan_runtime_enabled &&
        race_plan_start_us > 0 &&
        stop_us > race_plan_start_us &&
        isfinite(race_plan_track_distance_m) &&
        race_plan_track_distance_m > 0.0f) {
        const float elapsed_s = (float)(stop_us - race_plan_start_us) / 1000000.0f;
        if (elapsed_s > 0.0f) {
            average_mps = race_plan_track_distance_m / elapsed_s;
            nav_state.race_plan_average_speed_mps = average_mps;
        }
    }
    race_plan_start_us = 0;
    race_plan_track_distance_m = 0.0f;
    return average_mps;
}

static float set_navigation_stopped_with_aux(uint8_t active_aux_percent);

static float set_navigation_stopped(void)
{
    return set_navigation_stopped_with_aux(0);
}

static float set_navigation_stopped_with_aux(uint8_t active_aux_percent)
{
    const int64_t stop_us = esp_timer_get_time();
    float race_plan_average_mps = 0.0f;
    portENTER_CRITICAL(&state_mux);
    const float loop_hz = nav_state.loop_hz;
    race_plan_average_mps = finish_race_plan_average_locked(stop_us);
    race_plan_runtime_enabled = false;
    nav_state.running = false;
    nav_state.distance_m = 0.0f;
    nav_state.angle_error_rad = 0.0f;
    nav_state.steer_percent = 0.0f;
    nav_state.active_speed_percent = 0;
    nav_state.active_aux_percent = active_aux_percent <= 100 ? active_aux_percent : 100;
    nav_state.average_speed_mps = 0.0f;
    nav_state.max_speed_mps = speed_max_mps;
    nav_state.loop_hz = loop_hz;
    update_active_race_segment_locked(NULL);
    error_integral_rad_s = 0.0f;
    previous_error_rad = 0.0f;
    have_previous_error = false;
    portEXIT_CRITICAL(&state_mux);
    return race_plan_average_mps;
}

static void apply_race_plan_stop(uint16_t target_index, uint8_t map_slot)
{
    const uint8_t stop_aux_percent = 100;
    motors_set_percent(MOTORS_MOTOR_AUX, stop_aux_percent);
    motors_brake_drive();
    const float average_mps = set_navigation_stopped_with_aux(stop_aux_percent);
    ESP_LOGI(TAG,
             "Plano corrida parada alvo=%u slot=%u: freio TB6612 ativo, turbina=%u%%, vel_media=%.3fm/s, controle desligado",
             (unsigned int)target_index,
             (unsigned int)map_slot,
             (unsigned int)stop_aux_percent,
             average_mps);
}

static void race_plan_task(void *arg)
{
    (void)arg;
    bool have_led_type = false;
    uint8_t last_led_type = CONTROL_RACE_SEGMENT_NORMAL;
    uint32_t loop_count = 0;
    int64_t loop_window_start_us = esp_timer_get_time();

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        const int64_t now_us = esp_timer_get_time();
        ++loop_count;
        const int64_t window_us = now_us - loop_window_start_us;
        if (window_us >= 1000000) {
            const float loop_hz = ((float)loop_count * 1000000.0f) / (float)window_us;
            portENTER_CRITICAL(&state_mux);
            nav_state.race_plan_loop_hz = loop_hz;
            portEXIT_CRITICAL(&state_mux);
            loop_count = 0;
            loop_window_start_us = now_us;
        }

        control_navigation_state_t local = {0};
        bool runtime_enabled = false;
        int8_t direction = 1;
        portENTER_CRITICAL(&state_mux);
        runtime_enabled = race_plan_runtime_enabled;
        direction = race_plan_direction;
        local = nav_state;
        portEXIT_CRITICAL(&state_mux);

        if (!runtime_enabled || !local.running || local.mode != CONTROL_NAV_MODE_AUTO_TRACK) {
            have_led_type = false;
            continue;
        }

        control_map_progress_t progress = {0};
        if (!map_progress_from_encoder(&local, &progress)) {
            continue;
        }

        if (progress.complete) {
            motors_stop_all();
            const float average_mps = set_navigation_stopped();
            ESP_LOGI(TAG,
                     "Plano de corrida concluido slot=%u dist=%.3fm vel_media=%.3fm/s",
                     (unsigned int)local.map_slot,
                     progress.progress_m,
                     average_mps);
            continue;
        }

        control_race_plan_segment_t segment = {0};
        uint8_t segment_index = 0;
        const bool has_segment = race_plan_segment_for_target_index(progress.target_index, &segment, &segment_index);
        if (has_segment && segment.type == CONTROL_RACE_SEGMENT_STOP) {
            update_race_plan_led(CONTROL_RACE_SEGMENT_STOP);
            apply_race_plan_stop(progress.target_index, local.map_slot);
            continue;
        }
        const control_race_plan_segment_t effective_segment = has_segment ?
                                                                  race_plan_effective_segment(&segment,
                                                                                              segment_index,
                                                                                              progress.progress_m) :
                                                                  (control_race_plan_segment_t){0};

        if (has_segment && (!have_led_type || last_led_type != segment.type)) {
            update_race_plan_led((control_race_segment_type_t)segment.type);
            last_led_type = segment.type;
            have_led_type = true;
        }

        portENTER_CRITICAL(&state_mux);
        if (race_plan_runtime_enabled && nav_state.running && nav_state.mode == CONTROL_NAV_MODE_AUTO_TRACK) {
            nav_state.target_index = progress.target_index;
            nav_state.point_count = local.point_count;
            nav_state.target_x_m = progress.target_x_m;
            nav_state.target_y_m = progress.target_y_m;
            nav_state.distance_m = progress.distance_to_target_m;
            nav_state.map_x_m = progress.map_x_m;
            nav_state.map_y_m = progress.map_y_m;
            nav_state.map_heading_rad = progress.map_heading_rad;

            if (has_segment) {
                nav_state.speed_percent = (int8_t)signed_speed_percent(direction, effective_segment.speed_percent);
                nav_state.kp = effective_segment.kp;
                nav_state.ki = effective_segment.ki;
                nav_state.kd = effective_segment.kd;
                nav_state.motor_limit_percent = effective_segment.max_speed_percent;
                nav_state.aux_percent = effective_segment.aux_percent;
                nav_state.race_segment_active = true;
                nav_state.race_segment_type = segment.type;
                nav_state.race_segment_start_index = segment.start_index;
                nav_state.race_segment_end_index = segment.end_index;
                nav_state.race_segment_speed_percent = effective_segment.speed_percent;
                nav_state.race_segment_max_speed_percent = effective_segment.max_speed_percent;
                nav_state.race_segment_aux_percent = effective_segment.aux_percent;
            } else {
                update_active_race_segment_locked(NULL);
            }
        }
        portEXIT_CRITICAL(&state_mux);
    }
}

static void control_timer_cb(void *arg)
{
    (void)arg;

    if (control_task_handle != NULL) {
        xTaskNotifyGive(control_task_handle);
    }
}

static void race_plan_timer_cb(void *arg)
{
    (void)arg;

    if (race_plan_task_handle != NULL) {
        xTaskNotifyGive(race_plan_task_handle);
    }
}

static void track_odometry_timer_cb(void *arg)
{
    (void)arg;

    if (track_odometry_task_handle != NULL) {
        xTaskNotifyGive(track_odometry_task_handle);
    }
}

static void track_odometry_task(void *arg)
{
    (void)arg;
    uint32_t loop_count = 0;
    int64_t loop_window_start_us = esp_timer_get_time();

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const int64_t now_us = esp_timer_get_time();
        ++loop_count;
        const int64_t window_us = now_us - loop_window_start_us;
        if (window_us >= 1000000) {
            const float loop_hz = ((float)loop_count * 1000000.0f) / (float)window_us;
            portENTER_CRITICAL(&state_mux);
            nav_state.track_odometry_loop_hz = loop_hz;
            portEXIT_CRITICAL(&state_mux);
            loop_count = 0;
            loop_window_start_us = now_us;
        }
        encoder_sample_now();
        odometry_update_from_sensors();
        memory_maps_record_update();
    }
}

static void control_task(void *arg)
{
    (void)arg;
    uint32_t loop_count = 0;
    int64_t loop_window_start_us = esp_timer_get_time();
    int64_t last_loop_us = loop_window_start_us;
    int last_aux_percent = -1;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        const int64_t now_us = esp_timer_get_time();
        int64_t dt_us = now_us - last_loop_us;
        if (dt_us <= 0) {
            dt_us = CONTROL_TASK_PERIOD_US;
        }
        last_loop_us = now_us;

        ++loop_count;
        const int64_t window_us = now_us - loop_window_start_us;
        if (window_us >= 1000000) {
            const float loop_hz = ((float)loop_count * 1000000.0f) / (float)window_us;
            portENTER_CRITICAL(&state_mux);
            nav_state.loop_hz = loop_hz;
            portEXIT_CRITICAL(&state_mux);
            loop_count = 0;
            loop_window_start_us = now_us;
        }

        control_navigation_state_t local = {0};
        control_auto_track_config_t local_auto_config = {0};
        float local_manual_kp = 0.0f;
        float local_manual_ki = 0.0f;
        float local_manual_kd = 0.0f;
        bool local_race_plan_runtime_enabled = false;
        portENTER_CRITICAL(&state_mux);
        local = nav_state;
        local_auto_config = auto_track_config;
        local_manual_kp = manual_kp;
        local_manual_ki = manual_ki;
        local_manual_kd = manual_kd;
        local_race_plan_runtime_enabled = race_plan_runtime_enabled;
        portEXIT_CRITICAL(&state_mux);

        if (!local.running) {
            last_aux_percent = 0;
            continue;
        }
        if (!safety_motors_allowed()) {
            motors_stop_all_immediate();
            last_aux_percent = 0;
            set_navigation_stopped();
            ESP_LOGW(TAG, "Controle parado: bloqueio de seguranca ativo");
            continue;
        }

        if (local.mode == CONTROL_NAV_MODE_LINE || local.mode == CONTROL_NAV_MODE_AUTO_TRACK) {
            const bool auto_track_mode = local.mode == CONTROL_NAV_MODE_AUTO_TRACK;
            const bool race_plan_runtime_mode = auto_track_mode && local_race_plan_runtime_enabled;
            odometry_state_t speed_odometry = {0};
            const bool odometry_valid = odometry_get_state(&speed_odometry);
            if (odometry_valid) {
                update_speed_stats(speed_odometry.linear_mps);
            }
            if (auto_track_mode && !race_plan_runtime_mode && local.target_index >= local.point_count) {
                motors_stop_all();
                last_aux_percent = 0;
                set_navigation_stopped();
                ESP_LOGI(TAG, "Mapa concluido em auto pista slot=%u", (unsigned int)local.map_slot);
                continue;
            }

            line_sensor_state_t line = {0};
            const bool line_state_valid = line_sensor_get_state(&line);
            const bool use_previous_line_error = line_state_valid &&
                                                 line.calibrated_valid &&
                                                 !line.line_visible &&
                                                 have_previous_error;
            if (!line_state_valid || !line.calibrated_valid || (!line.line_visible && !use_previous_line_error)) {
                if (!auto_track_mode || race_plan_runtime_mode || !local_auto_config.line_loss_odometry_enabled) {
                    motors_stop_all_immediate();
                    last_aux_percent = 0;
                    portENTER_CRITICAL(&state_mux);
                    nav_state.distance_m = 0.0f;
                    nav_state.angle_error_rad = have_previous_error ? previous_error_rad : 0.0f;
                    nav_state.steer_percent = 0.0f;
                    nav_state.active_speed_percent = 0;
                    nav_state.active_aux_percent = 0;
                    portEXIT_CRITICAL(&state_mux);
                    continue;
                }
            } else {
                memory_map_point_t map_target = {0};
                float map_distance = local.distance_m;
                if (auto_track_mode && !race_plan_runtime_mode && local.target_index < local.point_count) {
                    map_target = active_map[local.target_index];
                }
                control_map_progress_t map_progress = {0};
                const bool map_progress_valid = auto_track_mode &&
                                                !race_plan_runtime_mode &&
                                                map_progress_from_encoder(&local, &map_progress);
                if (map_progress_valid) {
                    local.target_index = map_progress.target_index;
                    map_target.x_m = map_progress.target_x_m;
                    map_target.y_m = map_progress.target_y_m;
                    map_distance = map_progress.distance_to_target_m;
                    if (map_progress.complete) {
                        motors_stop_all();
                        last_aux_percent = 0;
                        set_navigation_stopped();
                        ESP_LOGI(TAG,
                                 "Ponto final alcancado por encoder em auto pista slot=%u dist=%.3fm",
                                 (unsigned int)local.map_slot,
                                 map_progress.progress_m);
                        continue;
                    }
                }

                const float dt_s = (float)dt_us / 1000000.0f;
                const float error = use_previous_line_error
                                        ? previous_error_rad
                                        : CONTROL_LINE_STEER_SIGN *
                                              (((float)line.position - CONTROL_LINE_CENTER_POSITION) /
                                               CONTROL_LINE_POSITION_SCALE);

                control_drive_selection_t drive = race_plan_runtime_mode || !auto_track_mode ?
                                                      select_line_drive(&local,
                                                                        local_manual_kp,
                                                                        local_manual_ki,
                                                                        local_manual_kd) :
                                                      select_drive_for_target(&local,
                                                                              true,
                                                                              local.target_index,
                                                                              local_manual_kp,
                                                                              local_manual_ki,
                                                                              local_manual_kd);
                if (drive.stop) {
                    apply_race_plan_stop(local.target_index, local.map_slot);
                    last_aux_percent = 100;
                    continue;
                }
                const int active_aux_percent = compensate_motor_percent(drive.profile.aux_percent,
                                                                        local.battery_compensation_enabled);
                if (active_aux_percent != last_aux_percent) {
                    motors_set_percent(MOTORS_MOTOR_AUX, active_aux_percent);
                    last_aux_percent = active_aux_percent;
                }

                update_error_integral(error, dt_s);
                const float derivative = (!use_previous_line_error && have_previous_error) ?
                                             (error - previous_error_rad) / dt_s :
                                             0.0f;
                previous_error_rad = error;
                have_previous_error = true;
                const float steer = clamp_abs((drive.profile.kp * error) +
                                                  (drive.profile.ki * error_integral_rad_s) +
                                                  (drive.profile.kd * derivative),
                                              CONTROL_MAX_STEER_PERCENT);
                const int left = clamp_motor_percent((float)drive.drive_speed_percent - steer,
                                                     drive.motor_limit_percent,
                                                     local.battery_compensation_enabled);
                const int right = clamp_motor_percent((float)drive.drive_speed_percent + steer,
                                                      drive.motor_limit_percent,
                                                      local.battery_compensation_enabled);

                motors_set_percent(MOTORS_MOTOR_LEFT, left);
                motors_set_percent(MOTORS_MOTOR_RIGHT, right);

                portENTER_CRITICAL(&state_mux);
                if (auto_track_mode && !race_plan_runtime_mode) {
                    nav_state.target_index = local.target_index;
                    nav_state.point_count = local.point_count;
                    nav_state.target_x_m = map_target.x_m;
                    nav_state.target_y_m = map_target.y_m;
                    nav_state.distance_m = map_distance;
                    if (map_progress_valid) {
                        nav_state.map_x_m = map_progress.map_x_m;
                        nav_state.map_y_m = map_progress.map_y_m;
                        nav_state.map_heading_rad = map_progress.map_heading_rad;
                    }
                } else if (!auto_track_mode) {
                    float display_position = CONTROL_LINE_CENTER_POSITION +
                                             ((error / CONTROL_LINE_STEER_SIGN) * CONTROL_LINE_POSITION_SCALE);
                    if (display_position < 0.0f) {
                        display_position = 0.0f;
                    }
                    if (display_position > 7000.0f) {
                        display_position = 7000.0f;
                    }
                    nav_state.target_index = (uint16_t)lroundf(display_position);
                    nav_state.point_count = 7000;
                    nav_state.target_x_m = display_position / 7000.0f;
                    nav_state.target_y_m = 0.0f;
                    nav_state.distance_m = fabsf(error);
                }
                nav_state.angle_error_rad = error;
                nav_state.steer_percent = steer;
                nav_state.kp = drive.profile.kp;
                nav_state.ki = drive.profile.ki;
                nav_state.kd = drive.profile.kd;
                nav_state.active_speed_percent = drive.selected_speed_percent;
                if (!race_plan_runtime_mode) {
                    update_active_race_segment_locked(auto_track_mode ? &drive : NULL);
                }
                nav_state.active_aux_percent = (uint8_t)active_aux_percent;
                portEXIT_CRITICAL(&state_mux);
                continue;
            }
        }

        odometry_state_t odometry = {0};
        if (!odometry_get_state(&odometry)) {
            continue;
        }
        update_speed_stats(odometry.linear_mps);

        if (local.target_index >= local.point_count) {
            motors_stop_all();
            last_aux_percent = 0;
            set_navigation_stopped();
            ESP_LOGI(TAG, "Mapa concluido slot=%u", (unsigned int)local.map_slot);
            continue;
        }

        memory_map_point_t target = active_map[local.target_index];
        float dx = target.x_m - odometry.x_m;
        float dy = target.y_m - odometry.y_m;
        float distance = sqrtf((dx * dx) + (dy * dy));

        while (distance <= CONTROL_TARGET_REACHED_DISTANCE_M &&
               local.target_index + 1 < local.point_count) {
            ++local.target_index;
            target = active_map[local.target_index];
            dx = target.x_m - odometry.x_m;
            dy = target.y_m - odometry.y_m;
            distance = sqrtf((dx * dx) + (dy * dy));
        }

        if (distance <= CONTROL_TARGET_REACHED_DISTANCE_M &&
            local.target_index + 1 >= local.point_count) {
            motors_stop_all();
            last_aux_percent = 0;
            set_navigation_stopped();
            ESP_LOGI(TAG, "Ponto final alcancado slot=%u", (unsigned int)local.map_slot);
            continue;
        }

        control_drive_selection_t drive = select_drive_for_target(&local,
                                                                  true,
                                                                  local.target_index,
                                                                  local_manual_kp,
                                                                  local_manual_ki,
                                                                  local_manual_kd);
        if (drive.stop) {
            apply_race_plan_stop(local.target_index, local.map_slot);
            last_aux_percent = 100;
            continue;
        }

        const int active_aux_percent = compensate_motor_percent(drive.profile.aux_percent,
                                                                local.battery_compensation_enabled);
        if (active_aux_percent != last_aux_percent) {
            motors_set_percent(MOTORS_MOTOR_AUX, active_aux_percent);
            last_aux_percent = active_aux_percent;
        }

        const float target_angle = atan2f(dy, dx);
        const float error = wrap_pi(target_angle - odometry.heading_rad);
        const float dt_s = (float)dt_us / 1000000.0f;
        update_error_integral(error, dt_s);
        const float derivative = have_previous_error ? wrap_pi(error - previous_error_rad) / dt_s : 0.0f;
        previous_error_rad = error;
        have_previous_error = true;
        const float steer = clamp_abs((drive.profile.kp * error) +
                                          (drive.profile.ki * error_integral_rad_s) +
                                          (drive.profile.kd * derivative),
                                      CONTROL_MAX_STEER_PERCENT);
        const int left = clamp_motor_percent((float)drive.drive_speed_percent - steer,
                                             drive.motor_limit_percent,
                                             local.battery_compensation_enabled);
        const int right = clamp_motor_percent((float)drive.drive_speed_percent + steer,
                                              drive.motor_limit_percent,
                                              local.battery_compensation_enabled);

        motors_set_percent(MOTORS_MOTOR_LEFT, left);
        motors_set_percent(MOTORS_MOTOR_RIGHT, right);

        portENTER_CRITICAL(&state_mux);
        nav_state.target_index = local.target_index;
        nav_state.target_x_m = target.x_m;
        nav_state.target_y_m = target.y_m;
        nav_state.distance_m = distance;
        nav_state.angle_error_rad = error;
        nav_state.steer_percent = steer;
        nav_state.kp = drive.profile.kp;
        nav_state.ki = drive.profile.ki;
        nav_state.kd = drive.profile.kd;
        nav_state.active_speed_percent = drive.selected_speed_percent;
        update_active_race_segment_locked(&drive);
        nav_state.active_aux_percent = (uint8_t)active_aux_percent;
        portEXIT_CRITICAL(&state_mux);
    }
}

esp_err_t control_init(void)
{
    if (initialized) {
        return ESP_OK;
    }

    esp_err_t ret = motors_init();
    if (ret != ESP_OK) {
        return ret;
    }
    memset(&nav_state, 0, sizeof(nav_state));
    manual_kp = CONTROL_HEADING_KP_PERCENT_PER_RAD;
    manual_ki = CONTROL_HEADING_KI_PERCENT_PER_RAD_S;
    manual_kd = CONTROL_HEADING_KD_PERCENT_S_PER_RAD;
    nav_state.kp = manual_kp;
    nav_state.ki = manual_ki;
    nav_state.kd = manual_kd;
    nav_state.motor_limit_percent = CONTROL_DEFAULT_MOTOR_LIMIT_PERCENT;
    nav_state.active_speed_percent = 0;
    nav_state.aux_percent = 0;
    nav_state.active_aux_percent = 0;
    nav_state.loop_hz = 0.0f;
    nav_state.race_plan_loop_hz = 0.0f;
    nav_state.track_odometry_loop_hz = 0.0f;
    nav_state.line_sensor_loop_hz = 0.0f;
    nav_state.imu_loop_hz = 0.0f;
    nav_state.speed_profile_enabled = false;
    nav_state.battery_compensation_enabled = false;
    reset_speed_stats();
    set_default_speed_profile();
    set_default_auto_track_config();
    load_settings_from_nvs();

    BaseType_t created = xTaskCreatePinnedToCore(control_task,
                                                 "control_task",
                                                 4096,
                                                 NULL,
                                                 CONTROL_TASK_PRIORITY,
                                                 &control_task_handle,
                                                 CONTROL_TASK_CORE_ID);
    if (created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    created = xTaskCreatePinnedToCore(race_plan_task,
                                      "race_plan_task",
                                      4096,
                                      NULL,
                                      CONTROL_RACE_PLAN_TASK_PRIORITY,
                                      &race_plan_task_handle,
                                      CONTROL_RACE_PLAN_TASK_CORE_ID);
    if (created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    created = xTaskCreatePinnedToCore(track_odometry_task,
                                      "track_odometry",
                                      4096,
                                      NULL,
                                      CONTROL_TRACK_ODOMETRY_TASK_PRIORITY,
                                      &track_odometry_task_handle,
                                      CONTROL_TRACK_ODOMETRY_TASK_CORE_ID);
    if (created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    const esp_timer_create_args_t timer_args = {
        .callback = control_timer_cb,
        .skip_unhandled_events = true,
        .name = "control_loop",
    };
    ret = esp_timer_create(&timer_args, &control_timer_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = esp_timer_start_periodic(control_timer_handle, CONTROL_TASK_PERIOD_US);
    if (ret != ESP_OK) {
        return ret;
    }

    const esp_timer_create_args_t race_plan_timer_args = {
        .callback = race_plan_timer_cb,
        .skip_unhandled_events = true,
        .name = "race_plan",
    };
    ret = esp_timer_create(&race_plan_timer_args, &race_plan_timer_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = esp_timer_start_periodic(race_plan_timer_handle, CONTROL_RACE_PLAN_TASK_PERIOD_US);
    if (ret != ESP_OK) {
        return ret;
    }

    const esp_timer_create_args_t track_odometry_timer_args = {
        .callback = track_odometry_timer_cb,
        .skip_unhandled_events = true,
        .name = "track_odom",
    };
    ret = esp_timer_create(&track_odometry_timer_args, &track_odometry_timer_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = esp_timer_start_periodic(track_odometry_timer_handle, CONTROL_TRACK_ODOMETRY_TASK_PERIOD_US);
    if (ret != ESP_OK) {
        return ret;
    }

    initialized = true;
    ESP_LOGI(TAG,
             "Controle de navegacao iniciado control=%.0fHz/%dus prio=%d race_plan=%.0fHz/%dus prio=%d track_odom=%.0fHz/%dus prio=%d core=%d",
             CONTROL_TASK_TARGET_HZ,
             CONTROL_TASK_PERIOD_US,
             CONTROL_TASK_PRIORITY,
             CONTROL_RACE_PLAN_TASK_TARGET_HZ,
             CONTROL_RACE_PLAN_TASK_PERIOD_US,
             CONTROL_RACE_PLAN_TASK_PRIORITY,
             CONTROL_TRACK_ODOMETRY_TASK_TARGET_HZ,
             CONTROL_TRACK_ODOMETRY_TASK_PERIOD_US,
             CONTROL_TRACK_ODOMETRY_TASK_PRIORITY,
             CONTROL_TASK_CORE_ID);
    return ESP_OK;
}

esp_err_t control_set_motor_percent(control_motor_id_t motor, int percent)
{
    return motors_set_percent((motors_motor_id_t)motor, percent);
}

esp_err_t control_stop_all(void)
{
    return control_emergency_stop();
}

esp_err_t control_emergency_stop(void)
{
    set_navigation_stopped();
    motors_set_percent(MOTORS_MOTOR_AUX, 0);
    return motors_brake_drive_for_ms(1000);
}

esp_err_t control_start_map(uint8_t map_slot, int8_t speed_percent)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (speed_percent == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t total_points = 0;
    esp_err_t ret = load_active_map(map_slot, &total_points);
    if (ret != ESP_OK) {
        return ret;
    }

    esp_err_t imu_ret = imu_reset_yaw();
    if (imu_ret != ESP_OK) {
        ESP_LOGW(TAG, "Falha ao resetar yaw da IMU no start do controle: %s", esp_err_to_name(imu_ret));
    }

    esp_err_t odom_heading_ret = odometry_reset_heading();
    if (odom_heading_ret != ESP_OK) {
        return odom_heading_ret;
    }

    esp_err_t odom_ret = odometry_set_position(active_map[0].x_m, active_map[0].y_m);
    if (odom_ret != ESP_OK) {
        return odom_ret;
    }
    reset_speed_stats();

    portENTER_CRITICAL(&state_mux);
    const float kp = manual_kp;
    const float ki = manual_ki;
    const float kd = manual_kd;
    const uint8_t motor_limit_percent = nav_state.motor_limit_percent;
    const uint8_t aux_percent = nav_state.aux_percent;
    const float loop_hz = nav_state.loop_hz;
    const bool speed_profile_enabled = nav_state.speed_profile_enabled;
    const bool battery_compensation_enabled = nav_state.battery_compensation_enabled;
    memset(&nav_state, 0, sizeof(nav_state));
    race_plan_runtime_enabled = false;
    race_plan_start_us = 0;
    race_plan_track_distance_m = 0.0f;
    nav_state.running = true;
    nav_state.mode = CONTROL_NAV_MODE_ODOMETRY;
    nav_state.map_slot = map_slot;
    nav_state.point_count = total_points;
    nav_state.speed_percent = speed_percent;
    nav_state.target_index = CONTROL_FIRST_MAP_TARGET_INDEX;
    nav_state.target_x_m = active_map[CONTROL_FIRST_MAP_TARGET_INDEX].x_m;
    nav_state.target_y_m = active_map[CONTROL_FIRST_MAP_TARGET_INDEX].y_m;
    nav_state.map_x_m = active_map[0].x_m;
    nav_state.map_y_m = active_map[0].y_m;
    nav_state.map_heading_rad = atan2f(active_map[CONTROL_FIRST_MAP_TARGET_INDEX].y_m - active_map[0].y_m,
                                       active_map[CONTROL_FIRST_MAP_TARGET_INDEX].x_m - active_map[0].x_m);
    nav_state.kp = kp;
    nav_state.ki = ki;
    nav_state.kd = kd;
    nav_state.active_speed_percent = abs_speed_percent(speed_percent);
    nav_state.motor_limit_percent = motor_limit_percent;
    nav_state.aux_percent = aux_percent;
    nav_state.active_aux_percent = 0;
    nav_state.loop_hz = loop_hz;
    nav_state.speed_profile_enabled = speed_profile_enabled;
    nav_state.battery_compensation_enabled = battery_compensation_enabled;
    error_integral_rad_s = 0.0f;
    previous_error_rad = 0.0f;
    have_previous_error = false;
    portEXIT_CRITICAL(&state_mux);
    reset_map_encoder_reference();

    ESP_LOGI(TAG,
             "Controle mapa iniciado slot=%u pontos=%u speed=%d%% origem=[%.3f %.3f] alvo=ponto%u[%.3f %.3f]",
             (unsigned int)map_slot,
             (unsigned int)total_points,
             (int)speed_percent,
             active_map[0].x_m,
             active_map[0].y_m,
             (unsigned int)CONTROL_FIRST_MAP_TARGET_INDEX,
             active_map[CONTROL_FIRST_MAP_TARGET_INDEX].x_m,
             active_map[CONTROL_FIRST_MAP_TARGET_INDEX].y_m);
    return ESP_OK;
}

esp_err_t control_start_line(int8_t speed_percent)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (speed_percent == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    line_sensor_state_t line = {0};
    if (!line_sensor_get_state(&line) || !line.calibrated_valid) {
        return ESP_ERR_INVALID_STATE;
    }
    reset_speed_stats();

    portENTER_CRITICAL(&state_mux);
    const float kp = manual_kp;
    const float ki = manual_ki;
    const float kd = manual_kd;
    const uint8_t motor_limit_percent = nav_state.motor_limit_percent;
    const uint8_t aux_percent = nav_state.aux_percent;
    const float loop_hz = nav_state.loop_hz;
    const bool speed_profile_enabled = nav_state.speed_profile_enabled;
    const bool battery_compensation_enabled = nav_state.battery_compensation_enabled;
    memset(&nav_state, 0, sizeof(nav_state));
    race_plan_runtime_enabled = false;
    race_plan_start_us = 0;
    race_plan_track_distance_m = 0.0f;
    nav_state.running = true;
    nav_state.mode = CONTROL_NAV_MODE_LINE;
    nav_state.map_slot = 0;
    nav_state.point_count = 7000;
    nav_state.target_index = line.position;
    nav_state.speed_percent = speed_percent;
    nav_state.target_x_m = ((float)line.position / 7000.0f);
    nav_state.target_y_m = 0.0f;
    nav_state.kp = kp;
    nav_state.ki = ki;
    nav_state.kd = kd;
    nav_state.active_speed_percent = abs_speed_percent(speed_percent);
    nav_state.motor_limit_percent = motor_limit_percent;
    nav_state.aux_percent = aux_percent;
    nav_state.active_aux_percent = 0;
    nav_state.loop_hz = loop_hz;
    nav_state.speed_profile_enabled = speed_profile_enabled;
    nav_state.battery_compensation_enabled = battery_compensation_enabled;
    error_integral_rad_s = 0.0f;
    previous_error_rad = 0.0f;
    have_previous_error = false;
    portEXIT_CRITICAL(&state_mux);

    ESP_LOGI(TAG, "Controle linha iniciado speed=%d%% pos=%u", (int)speed_percent, (unsigned int)line.position);
    return ESP_OK;
}

esp_err_t control_start_auto_track(uint8_t map_slot, int8_t speed_percent)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (speed_percent == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    line_sensor_state_t line = {0};
    if (!line_sensor_get_state(&line) || !line.calibrated_valid) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t total_points = 0;
    esp_err_t ret = load_active_map(map_slot, &total_points);
    if (ret != ESP_OK) {
        return ret;
    }

    esp_err_t imu_ret = imu_reset_yaw();
    if (imu_ret != ESP_OK) {
        ESP_LOGW(TAG, "Falha ao resetar yaw da IMU no start auto pista: %s", esp_err_to_name(imu_ret));
    }

    esp_err_t odom_heading_ret = odometry_reset_heading();
    if (odom_heading_ret != ESP_OK) {
        return odom_heading_ret;
    }

    esp_err_t odom_ret = odometry_set_position(active_map[0].x_m, active_map[0].y_m);
    if (odom_ret != ESP_OK) {
        return odom_ret;
    }
    reset_speed_stats();

    float race_plan_distance_m = 0.0f;
    if (total_points > 1) {
        race_plan_distance_m = active_map_distance_m[total_points - 1];
        if (!isfinite(race_plan_distance_m) || race_plan_distance_m <= 0.0f) {
            race_plan_distance_m = 0.0f;
        }
    }
    const int64_t start_us = esp_timer_get_time();

    bool start_race_plan_runtime = false;
    portENTER_CRITICAL(&state_mux);
    const float kp = manual_kp;
    const float ki = manual_ki;
    const float kd = manual_kd;
    const uint8_t motor_limit_percent = nav_state.motor_limit_percent;
    const uint8_t aux_percent = nav_state.aux_percent;
    const float loop_hz = nav_state.loop_hz;
    const bool speed_profile_enabled = nav_state.speed_profile_enabled;
    const bool battery_compensation_enabled = nav_state.battery_compensation_enabled;
    memset(&nav_state, 0, sizeof(nav_state));
    start_race_plan_runtime = race_plan_enabled;
    race_plan_runtime_enabled = start_race_plan_runtime;
    race_plan_direction = speed_percent < 0 ? -1 : 1;
    race_plan_start_us = start_race_plan_runtime ? start_us : 0;
    race_plan_track_distance_m = start_race_plan_runtime ? race_plan_distance_m : 0.0f;
    nav_state.running = true;
    nav_state.mode = CONTROL_NAV_MODE_AUTO_TRACK;
    nav_state.map_slot = map_slot;
    nav_state.point_count = total_points;
    nav_state.speed_percent = speed_percent;
    nav_state.target_index = CONTROL_FIRST_MAP_TARGET_INDEX;
    nav_state.target_x_m = active_map[CONTROL_FIRST_MAP_TARGET_INDEX].x_m;
    nav_state.target_y_m = active_map[CONTROL_FIRST_MAP_TARGET_INDEX].y_m;
    nav_state.map_x_m = active_map[0].x_m;
    nav_state.map_y_m = active_map[0].y_m;
    nav_state.map_heading_rad = atan2f(active_map[CONTROL_FIRST_MAP_TARGET_INDEX].y_m - active_map[0].y_m,
                                       active_map[CONTROL_FIRST_MAP_TARGET_INDEX].x_m - active_map[0].x_m);
    nav_state.kp = kp;
    nav_state.ki = ki;
    nav_state.kd = kd;
    nav_state.active_speed_percent = abs_speed_percent(speed_percent);
    nav_state.motor_limit_percent = motor_limit_percent;
    nav_state.aux_percent = aux_percent;
    nav_state.active_aux_percent = 0;
    nav_state.loop_hz = loop_hz;
    nav_state.speed_profile_enabled = speed_profile_enabled;
    nav_state.battery_compensation_enabled = battery_compensation_enabled;
    error_integral_rad_s = 0.0f;
    previous_error_rad = 0.0f;
    have_previous_error = false;
    portEXIT_CRITICAL(&state_mux);
    reset_map_encoder_reference();

    ESP_LOGI(TAG,
             "Auto pista iniciado slot=%u pontos=%u speed=%d%% plano_task=%u origem=[%.3f %.3f] alvo=ponto%u[%.3f %.3f]",
             (unsigned int)map_slot,
             (unsigned int)total_points,
             (int)speed_percent,
             race_plan_enabled ? 1U : 0U,
             active_map[0].x_m,
             active_map[0].y_m,
             (unsigned int)CONTROL_FIRST_MAP_TARGET_INDEX,
             active_map[CONTROL_FIRST_MAP_TARGET_INDEX].x_m,
             active_map[CONTROL_FIRST_MAP_TARGET_INDEX].y_m);
    return ESP_OK;
}

esp_err_t control_stop_navigation(void)
{
    set_navigation_stopped();
    motors_set_percent(MOTORS_MOTOR_AUX, 0);
    return motors_brake_drive_for_ms(1000);
}

esp_err_t control_set_pid(float kp, float ki, float kd)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
        kp < 0.0f || ki < 0.0f || kd < 0.0f ||
        kp > CONTROL_MAX_PID_GAIN || ki > CONTROL_MAX_PID_GAIN || kd > CONTROL_MAX_PID_GAIN) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&state_mux);
    manual_kp = kp;
    manual_ki = ki;
    manual_kd = kd;
    if (!nav_state.speed_profile_enabled || !nav_state.running) {
        nav_state.kp = kp;
        nav_state.ki = ki;
        nav_state.kd = kd;
    }
    error_integral_rad_s = 0.0f;
    previous_error_rad = 0.0f;
    have_previous_error = false;
    portEXIT_CRITICAL(&state_mux);

    ESP_LOGI(TAG, "PID controle ajustado kp=%.3f ki=%.3f kd=%.3f", kp, ki, kd);
    return ESP_OK;
}

esp_err_t control_save_pid_settings(float kp, float ki, float kd, uint8_t limit_percent, uint8_t aux_percent)
{
    esp_err_t ret = control_set_pid(kp, ki, kd);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = control_set_motor_limit(limit_percent);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = control_set_aux_percent(aux_percent);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = save_settings_to_nvs(kp, ki, kd, limit_percent, aux_percent);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Falha ao salvar PID/limite: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG,
                 "PID/limite/turbina salvos kp=%.3f ki=%.3f kd=%.3f limit=%u aux=%u",
                 kp,
                 ki,
                 kd,
                 (unsigned int)limit_percent,
                 (unsigned int)aux_percent);
    }
    return ret;
}

esp_err_t control_set_speed_profile(const control_speed_profile_point_t *points, size_t count)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    (void)points;
    (void)count;
    ESP_LOGI(TAG, "Curva velocidade de controle removida; comando de perfil ignorado");
    return ESP_OK;
}

esp_err_t control_set_speed_profile_enabled(bool enabled)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&state_mux);
    nav_state.speed_profile_enabled = false;
    if (!enabled) {
        nav_state.kp = manual_kp;
        nav_state.ki = manual_ki;
        nav_state.kd = manual_kd;
    }
    error_integral_rad_s = 0.0f;
    previous_error_rad = 0.0f;
    have_previous_error = false;
    portEXIT_CRITICAL(&state_mux);

    esp_err_t ret = save_speed_profile_enabled_to_nvs(false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Falha ao salvar uso da curva velocidade: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "Curva velocidade de controle removida; comando ignorado enabled=%u", enabled ? 1U : 0U);
    return ret;
}

esp_err_t control_set_battery_compensation_enabled(bool enabled)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    bool update_aux_now = false;
    uint8_t aux_percent = 0;
    portENTER_CRITICAL(&state_mux);
    nav_state.battery_compensation_enabled = enabled;
    update_aux_now = nav_state.running && !nav_state.speed_profile_enabled;
    aux_percent = nav_state.aux_percent;
    portEXIT_CRITICAL(&state_mux);

    if (update_aux_now) {
        const int active_aux_percent = compensate_motor_percent(aux_percent, enabled);
        portENTER_CRITICAL(&state_mux);
        nav_state.active_aux_percent = (uint8_t)active_aux_percent;
        portEXIT_CRITICAL(&state_mux);
        motors_set_percent(MOTORS_MOTOR_AUX, active_aux_percent);
    }

    esp_err_t ret = save_battery_compensation_enabled_to_nvs(enabled);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Falha ao salvar compensacao bateria: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "Compensacao bateria %s", enabled ? "habilitada" : "desabilitada");
    return ret;
}

esp_err_t control_set_auto_track_config(const control_auto_track_config_t *config)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    control_auto_track_config_t sanitized = *config;
    sanitized.line_loss_odometry_enabled = sanitized.line_loss_odometry_enabled ? 1 : 0;

    portENTER_CRITICAL(&state_mux);
    auto_track_config = sanitized;
    portEXIT_CRITICAL(&state_mux);

    esp_err_t ret = save_auto_track_config_to_nvs();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Falha ao salvar plano auto pista: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG,
             "Config auto pista odom_sem_linha=%u",
             sanitized.line_loss_odometry_enabled ? 1U : 0U);
    return ret;
}

esp_err_t control_set_race_plan(const control_race_plan_segment_t *segments, size_t count, bool enabled)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!enabled) {
        portENTER_CRITICAL(&state_mux);
        memset(race_plan_segments, 0, sizeof(race_plan_segments));
        race_plan_segment_count = 0;
        race_plan_enabled = false;
        race_plan_runtime_enabled = false;
        race_plan_start_us = 0;
        race_plan_track_distance_m = 0.0f;
        portEXIT_CRITICAL(&state_mux);
        ESP_LOGI(TAG, "Plano de corrida desabilitado");
        return ESP_OK;
    }
    if (!race_plan_is_valid(segments, count) || count == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const int64_t start_us = esp_timer_get_time();

    portENTER_CRITICAL(&state_mux);
    memset(race_plan_segments, 0, sizeof(race_plan_segments));
    memcpy(race_plan_segments, segments, count * sizeof(control_race_plan_segment_t));
    race_plan_segment_count = (uint8_t)count;
    race_plan_enabled = true;
    if (nav_state.running && nav_state.mode == CONTROL_NAV_MODE_AUTO_TRACK) {
        race_plan_runtime_enabled = true;
        race_plan_direction = nav_state.speed_percent < 0 ? -1 : 1;
        race_plan_start_us = start_us;
        race_plan_track_distance_m = nav_state.point_count > 1 ?
                                         active_map_distance_m[nav_state.point_count - 1] :
                                         0.0f;
        if (!isfinite(race_plan_track_distance_m) || race_plan_track_distance_m <= 0.0f) {
            race_plan_track_distance_m = 0.0f;
        }
    }
    error_integral_rad_s = 0.0f;
    previous_error_rad = 0.0f;
    have_previous_error = false;
    portEXIT_CRITICAL(&state_mux);

    ESP_LOGI(TAG, "Plano de corrida aplicado segmentos=%u", (unsigned int)count);
    for (size_t i = 0; i < count; ++i) {
        const control_race_plan_segment_t *segment = &segments[i];
        ESP_LOGI(TAG,
                 "Plano segmento[%u] start=%u end=%u type=%u speed=%u max=%u aux=%u kp=%.3f ki=%.3f kd=%.3f",
                 (unsigned int)i,
                 (unsigned int)segment->start_index,
                 (unsigned int)segment->end_index,
                 (unsigned int)segment->type,
                 (unsigned int)segment->speed_percent,
                 (unsigned int)segment->max_speed_percent,
                 (unsigned int)segment->aux_percent,
                 segment->kp,
                 segment->ki,
                 segment->kd);
    }
    return ESP_OK;
}

esp_err_t control_set_motor_limit(uint8_t limit_percent)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (limit_percent > 100) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&state_mux);
    nav_state.motor_limit_percent = limit_percent;
    portEXIT_CRITICAL(&state_mux);

    ESP_LOGI(TAG, "Limite motor controle ajustado para %u%%", (unsigned int)limit_percent);
    return ESP_OK;
}

esp_err_t control_set_aux_percent(uint8_t aux_percent)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (aux_percent > 100) {
        return ESP_ERR_INVALID_ARG;
    }

    bool battery_compensation_enabled = false;
    portENTER_CRITICAL(&state_mux);
    nav_state.aux_percent = aux_percent;
    const bool apply_now = nav_state.running && !nav_state.speed_profile_enabled;
    battery_compensation_enabled = nav_state.battery_compensation_enabled;
    portEXIT_CRITICAL(&state_mux);

    if (apply_now) {
        const int active_aux_percent = compensate_motor_percent(aux_percent, battery_compensation_enabled);
        portENTER_CRITICAL(&state_mux);
        nav_state.active_aux_percent = (uint8_t)active_aux_percent;
        portEXIT_CRITICAL(&state_mux);
        motors_set_percent(MOTORS_MOTOR_AUX, active_aux_percent);
    }

    ESP_LOGI(TAG, "Turbina controle ajustada para %u%%", (unsigned int)aux_percent);
    return ESP_OK;
}

bool control_get_navigation_state(control_navigation_state_t *out_state)
{
    if (out_state == NULL || !initialized) {
        return false;
    }

    portENTER_CRITICAL(&state_mux);
    *out_state = nav_state;
    portEXIT_CRITICAL(&state_mux);
    return true;
}
