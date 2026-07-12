#include "odometry.h"

#include <math.h>
#include <string.h>

#include "encoder.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "imu.h"
#include "odometry_config.h"

static const char *TAG = "odometry";

static odometry_state_t state;
static portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;
static int32_t last_left_count;
static int32_t last_right_count;
static int64_t last_update_us;
static uint32_t rejected_delta_count;
static bool have_last_counts;
static bool initialized;

static float wrap_pi(float angle)
{
    while (angle > ODOMETRY_PI_F) {
        angle -= 2.0f * ODOMETRY_PI_F;
    }
    while (angle < -ODOMETRY_PI_F) {
        angle += 2.0f * ODOMETRY_PI_F;
    }
    return angle;
}

static float encoder_delta_to_meters(int32_t delta)
{
    return ((float)delta * ODOMETRY_WHEEL_CIRCUMFERENCE_M) /
           ODOMETRY_ENCODER_COUNTS_PER_WHEEL_REV;
}

static void integrate_pose(float *x_m, float *y_m, float *heading_rad, float ds, float new_heading_rad)
{
    const float previous_heading = *heading_rad;
    const float heading_delta = wrap_pi(new_heading_rad - previous_heading);
    const float heading_mid = wrap_pi(previous_heading + (heading_delta * 0.5f));
    *x_m += ds * cosf(heading_mid);
    *y_m += ds * sinf(heading_mid);
    *heading_rad = new_heading_rad;
}

static void apply_selected_pose_locked(void)
{
    state.x_m = state.fused_x_m;
    state.y_m = state.fused_y_m;
    state.heading_rad = state.fused_heading_rad;
}

static bool odometry_delta_is_valid(float left_m, float right_m, float encoder_dtheta, float dt_s)
{
    if (!isfinite(left_m) || !isfinite(right_m) || !isfinite(encoder_dtheta) || !isfinite(dt_s)) {
        return false;
    }

    float wheel_limit_m = (ODOMETRY_MAX_WHEEL_SPEED_MPS * dt_s) + ODOMETRY_SPIKE_MARGIN_M;
    if (wheel_limit_m > ODOMETRY_MAX_WHEEL_STEP_M) {
        wheel_limit_m = ODOMETRY_MAX_WHEEL_STEP_M;
    }
    if (wheel_limit_m < ODOMETRY_SPIKE_MARGIN_M) {
        wheel_limit_m = ODOMETRY_SPIKE_MARGIN_M;
    }

    return fabsf(left_m) <= wheel_limit_m &&
           fabsf(right_m) <= wheel_limit_m &&
           fabsf(encoder_dtheta) <= ODOMETRY_MAX_HEADING_STEP_RAD;
}

esp_err_t odometry_update_from_sensors(void)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    encoder_state_t encoder = {0};
    if (!encoder_get_state(&encoder)) {
        return ESP_ERR_INVALID_STATE;
    }

    const int64_t now_us = esp_timer_get_time();
    int64_t dt_us = now_us - last_update_us;
    if (dt_us <= 0) {
        dt_us = ODOMETRY_UPDATE_PERIOD_US;
    }
    last_update_us = now_us;

    const int32_t left_count = encoder.counts[ENCODER_LEFT];
    const int32_t right_count = encoder.counts[ENCODER_RIGHT];

    if (!have_last_counts) {
        last_left_count = left_count;
        last_right_count = right_count;
        have_last_counts = true;
        return ESP_OK;
    }

    const int32_t delta_left_count = left_count - last_left_count;
    const int32_t delta_right_count = right_count - last_right_count;
    last_left_count = left_count;
    last_right_count = right_count;

    const float left_m = encoder_delta_to_meters(delta_left_count);
    const float right_m = encoder_delta_to_meters(delta_right_count);
    const float ds = (left_m + right_m) * 0.5f;
    const float encoder_dtheta = (right_m - left_m) / ODOMETRY_TRACK_WIDTH_M;
    const float dt_s = (float)dt_us / 1000000.0f;

    if (!odometry_delta_is_valid(left_m, right_m, encoder_dtheta, dt_s)) {
        ++rejected_delta_count;
        portENTER_CRITICAL(&state_mux);
        state.linear_mps = 0.0f;
        state.angular_rad_s = 0.0f;
        state.left_count = left_count;
        state.right_count = right_count;
        portEXIT_CRITICAL(&state_mux);
        if (rejected_delta_count <= 5 || (rejected_delta_count % 50U) == 0U) {
            ESP_LOGW(TAG,
                     "Delta odometria ignorado #%u dt=%.6fs left=%ld right=%ld left_m=%.4f right_m=%.4f dtheta=%.3f",
                     (unsigned int)rejected_delta_count,
                     dt_s,
                     (long)delta_left_count,
                     (long)delta_right_count,
                     left_m,
                     right_m,
                     encoder_dtheta);
        }
        return ESP_OK;
    }

    imu_state_t imu = {0};
    const bool has_imu = imu_get_state(&imu);

    portENTER_CRITICAL(&state_mux);
    const float encoder_heading = wrap_pi(state.encoder_heading_rad + encoder_dtheta);
    float imu_heading = encoder_heading;
    float fused_heading = encoder_heading;
    float angular_rad_s = dt_s > 0.0f ? encoder_dtheta / dt_s : 0.0f;

    if (has_imu) {
        imu_heading = wrap_pi(imu.yaw_deg * (ODOMETRY_PI_F / 180.0f));
        angular_rad_s = imu.gyro_dps[2] * (ODOMETRY_PI_F / 180.0f);
        const float predicted_fused_heading = wrap_pi(state.fused_heading_rad + encoder_dtheta);
        fused_heading = wrap_pi(predicted_fused_heading +
                                (wrap_pi(imu_heading - predicted_fused_heading) * ODOMETRY_FUSION_IMU_GAIN));
    }

    integrate_pose(&state.encoder_x_m, &state.encoder_y_m, &state.encoder_heading_rad, ds, encoder_heading);
    integrate_pose(&state.imu_x_m, &state.imu_y_m, &state.imu_heading_rad, ds, imu_heading);
    integrate_pose(&state.fused_x_m, &state.fused_y_m, &state.fused_heading_rad, ds, fused_heading);
    apply_selected_pose_locked();
    state.linear_mps = dt_s > 0.0f ? ds / dt_s : 0.0f;
    state.angular_rad_s = angular_rad_s;
    state.left_count = left_count;
    state.right_count = right_count;
    state.imu_available = has_imu;
    portEXIT_CRITICAL(&state_mux);
    return ESP_OK;
}

esp_err_t odometry_init(void)
{
    if (initialized) {
        return ESP_OK;
    }

    memset(&state, 0, sizeof(state));
    have_last_counts = false;
    last_update_us = esp_timer_get_time();
    rejected_delta_count = 0;

    initialized = true;
    ESP_LOGI(TAG,
             "Odometria iniciada wheel=%.3fm track=%.3fm cpr=%.1f update=%.0fHz/%dus source=track_odometry",
             ODOMETRY_WHEEL_DIAMETER_M,
             ODOMETRY_TRACK_WIDTH_M,
             ODOMETRY_ENCODER_COUNTS_PER_WHEEL_REV,
             1000000.0f / (float)ODOMETRY_UPDATE_PERIOD_US,
             ODOMETRY_UPDATE_PERIOD_US);
    return ESP_OK;
}

esp_err_t odometry_reset(void)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    encoder_state_t encoder = {0};
    if (encoder_get_state(&encoder)) {
        last_left_count = encoder.counts[ENCODER_LEFT];
        last_right_count = encoder.counts[ENCODER_RIGHT];
        have_last_counts = true;
    } else {
        have_last_counts = false;
    }
    last_update_us = esp_timer_get_time();
    rejected_delta_count = 0;

    portENTER_CRITICAL(&state_mux);
    memset(&state, 0, sizeof(state));
    apply_selected_pose_locked();
    portEXIT_CRITICAL(&state_mux);

    ESP_LOGI(TAG, "Odometria resetada");
    return ESP_OK;
}

esp_err_t odometry_reset_heading(void)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    encoder_state_t encoder = {0};
    if (encoder_get_state(&encoder)) {
        last_left_count = encoder.counts[ENCODER_LEFT];
        last_right_count = encoder.counts[ENCODER_RIGHT];
        have_last_counts = true;
    }
    last_update_us = esp_timer_get_time();
    rejected_delta_count = 0;

    portENTER_CRITICAL(&state_mux);
    state.heading_rad = 0.0f;
    state.angular_rad_s = 0.0f;
    state.encoder_heading_rad = 0.0f;
    state.imu_heading_rad = 0.0f;
    state.fused_heading_rad = 0.0f;
    apply_selected_pose_locked();
    portEXIT_CRITICAL(&state_mux);

    ESP_LOGI(TAG, "Heading da odometria resetado para 0");
    return ESP_OK;
}

esp_err_t odometry_set_position(float x_m, float y_m)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    encoder_state_t encoder = {0};
    if (encoder_get_state(&encoder)) {
        last_left_count = encoder.counts[ENCODER_LEFT];
        last_right_count = encoder.counts[ENCODER_RIGHT];
        have_last_counts = true;
    } else {
        have_last_counts = false;
    }
    last_update_us = esp_timer_get_time();
    rejected_delta_count = 0;

    portENTER_CRITICAL(&state_mux);
    state.x_m = x_m;
    state.y_m = y_m;
    state.encoder_x_m = x_m;
    state.encoder_y_m = y_m;
    state.imu_x_m = x_m;
    state.imu_y_m = y_m;
    state.fused_x_m = x_m;
    state.fused_y_m = y_m;
    state.linear_mps = 0.0f;
    state.left_count = last_left_count;
    state.right_count = last_right_count;
    apply_selected_pose_locked();
    portEXIT_CRITICAL(&state_mux);

    ESP_LOGI(TAG, "Posicao da odometria ajustada para x=%.3f y=%.3f", x_m, y_m);
    return ESP_OK;
}

esp_err_t odometry_set_pose_source(odometry_pose_source_t source)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    (void)source;
    portENTER_CRITICAL(&state_mux);
    apply_selected_pose_locked();
    portEXIT_CRITICAL(&state_mux);

    ESP_LOGI(TAG, "Fonte da pose da odometria=Combinado");
    return ESP_OK;
}

odometry_pose_source_t odometry_get_pose_source(void)
{
    return ODOMETRY_POSE_SOURCE_FUSED;
}

bool odometry_get_state(odometry_state_t *out_state)
{
    if (out_state == NULL || !initialized) {
        return false;
    }

    portENTER_CRITICAL(&state_mux);
    *out_state = state;
    portEXIT_CRITICAL(&state_mux);
    return true;
}
