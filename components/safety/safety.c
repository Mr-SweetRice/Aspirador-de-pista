#include "safety.h"

#include <math.h>
#include <string.h>

#include "battery_level.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.h"
#include "line_sensor.h"
#include "memory_config.h"
#include "nvs.h"

#define SAFETY_NVS_KEY_COLLISION_EN "safe_col_en"
#define SAFETY_NVS_KEY_BATTERY_EN "safe_bat_en"
#define SAFETY_NVS_KEY_LINE_EN "safe_line_en"
#define SAFETY_NVS_KEY_BLE_EN "safe_ble_en"
#define SAFETY_NVS_KEY_ROLL_MDEG "safe_roll_md"
#define SAFETY_NVS_KEY_BATT_CENTI "safe_bat_cp"
#define SAFETY_NVS_KEY_LINE_MS "safe_line_ms"
#define SAFETY_DEFAULT_ROLL_LIMIT_DEG 6.0f
#define SAFETY_DEFAULT_BATTERY_BLOCK_PERCENT 10.0f
#define SAFETY_BATTERY_PRESENT_MIN_V 6.0f
#define SAFETY_DEFAULT_LINE_LOSS_TIMEOUT_S 1.0f
#define SAFETY_TASK_PERIOD_MS 50
#define SAFETY_ROLL_FILTER_MS 200
#define SAFETY_ROLL_FILTER_SAMPLES (SAFETY_ROLL_FILTER_MS / SAFETY_TASK_PERIOD_MS)
#define SAFETY_TASK_CORE_ID 1
#define SAFETY_TASK_PRIORITY 6

static const char *TAG = "safety";
static portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;
static safety_state_t state;
static TaskHandle_t safety_task_handle;
static bool initialized;
static float roll_samples[SAFETY_ROLL_FILTER_SAMPLES];
static uint8_t roll_sample_index;
static uint8_t roll_sample_count;
static float roll_sample_sum;
static int64_t line_lost_since_us;

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static esp_err_t save_settings_to_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    safety_state_t local = {0};
    portENTER_CRITICAL(&state_mux);
    local = state;
    portEXIT_CRITICAL(&state_mux);

    if (ret == ESP_OK) {
        ret = nvs_set_u8(handle, SAFETY_NVS_KEY_COLLISION_EN, local.collision_enabled ? 1 : 0);
    }
    if (ret == ESP_OK) {
        ret = nvs_set_u8(handle, SAFETY_NVS_KEY_BATTERY_EN, local.battery_block_enabled ? 1 : 0);
    }
    if (ret == ESP_OK) {
        ret = nvs_set_u8(handle, SAFETY_NVS_KEY_LINE_EN, local.line_loss_enabled ? 1 : 0);
    }
    if (ret == ESP_OK) {
        ret = nvs_set_u8(handle, SAFETY_NVS_KEY_BLE_EN, local.ble_loss_enabled ? 1 : 0);
    }
    if (ret == ESP_OK) {
        ret = nvs_set_i32(handle, SAFETY_NVS_KEY_ROLL_MDEG, (int32_t)lroundf(local.roll_limit_deg * 1000.0f));
    }
    if (ret == ESP_OK) {
        ret = nvs_set_i32(handle, SAFETY_NVS_KEY_BATT_CENTI, (int32_t)lroundf(local.battery_block_percent * 100.0f));
    }
    if (ret == ESP_OK) {
        ret = nvs_set_i32(handle, SAFETY_NVS_KEY_LINE_MS, (int32_t)lroundf(local.line_loss_timeout_s * 1000.0f));
    }
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    return ret;
}

static void load_settings_from_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        return;
    }

    uint8_t enabled = 1;
    if (nvs_get_u8(handle, SAFETY_NVS_KEY_COLLISION_EN, &enabled) == ESP_OK) {
        state.collision_enabled = enabled != 0;
    }
    enabled = 1;
    if (nvs_get_u8(handle, SAFETY_NVS_KEY_BATTERY_EN, &enabled) == ESP_OK) {
        state.battery_block_enabled = enabled != 0;
    }
    enabled = 1;
    if (nvs_get_u8(handle, SAFETY_NVS_KEY_LINE_EN, &enabled) == ESP_OK) {
        state.line_loss_enabled = enabled != 0;
    }
    enabled = 1;
    if (nvs_get_u8(handle, SAFETY_NVS_KEY_BLE_EN, &enabled) == ESP_OK) {
        state.ble_loss_enabled = enabled != 0;
    }
    int32_t stored = 0;
    if (nvs_get_i32(handle, SAFETY_NVS_KEY_ROLL_MDEG, &stored) == ESP_OK) {
        state.roll_limit_deg = clamp_float((float)stored / 1000.0f, 1.0f, 90.0f);
    }
    if (nvs_get_i32(handle, SAFETY_NVS_KEY_BATT_CENTI, &stored) == ESP_OK) {
        state.battery_block_percent = clamp_float((float)stored / 100.0f, 0.0f, 100.0f);
    }
    if (nvs_get_i32(handle, SAFETY_NVS_KEY_LINE_MS, &stored) == ESP_OK) {
        state.line_loss_timeout_s = clamp_float((float)stored / 1000.0f, 0.1f, 10.0f);
    }

    nvs_close(handle);
}

static float update_roll_average(float roll_deg)
{
    if (roll_sample_count < SAFETY_ROLL_FILTER_SAMPLES) {
        roll_samples[roll_sample_index] = roll_deg;
        roll_sample_sum += roll_deg;
        ++roll_sample_count;
    } else {
        roll_sample_sum -= roll_samples[roll_sample_index];
        roll_samples[roll_sample_index] = roll_deg;
        roll_sample_sum += roll_deg;
    }
    roll_sample_index = (uint8_t)((roll_sample_index + 1U) % SAFETY_ROLL_FILTER_SAMPLES);
    return roll_sample_count > 0 ? roll_sample_sum / (float)roll_sample_count : roll_deg;
}

static void safety_task(void *arg)
{
    (void)arg;

    while (true) {
        imu_state_t imu = {0};
        battery_level_state_t battery = {0};
        line_sensor_state_t line = {0};
        const bool has_imu = imu_get_state(&imu);
        const bool has_battery_sample = battery_level_get_state(&battery) && battery.valid;
        const bool has_battery = has_battery_sample && battery.voltage_v >= SAFETY_BATTERY_PRESENT_MIN_V;
        const bool has_line = line_sensor_get_state(&line) && line.calibrated_valid;
        const int64_t now_us = esp_timer_get_time();
        const float filtered_roll = has_imu ? update_roll_average(imu.roll_deg) : state.current_roll_deg;
        float line_loss_elapsed_s = 0.0f;
        bool line_seen_once = state.line_seen_once;
        bool line_loss_active = false;

        if (has_line && line.line_visible) {
            line_seen_once = true;
            line_lost_since_us = 0;
        } else if (has_line && line_seen_once) {
            if (line_lost_since_us == 0) {
                line_lost_since_us = now_us;
            }
            line_loss_elapsed_s = (float)(now_us - line_lost_since_us) / 1000000.0f;
            line_loss_active = line_loss_elapsed_s >= state.line_loss_timeout_s;
        } else {
            line_lost_since_us = 0;
        }

        portENTER_CRITICAL(&state_mux);
        if (has_imu) {
            state.current_roll_deg = filtered_roll;
        }
        if (has_battery_sample) {
            state.current_battery_percent = battery.percent;
        }
        state.line_visible = has_line && line.line_visible;
        state.line_seen_once = line_seen_once;
        state.line_loss_elapsed_s = line_loss_elapsed_s;
        state.collision_active = state.collision_enabled && has_imu &&
                                 fabsf(state.current_roll_deg) > state.roll_limit_deg;
        state.battery_block_active = state.battery_block_enabled && has_battery &&
                                     state.current_battery_percent <= state.battery_block_percent;
        state.line_loss_active = state.line_loss_enabled && line_loss_active;
        state.ble_loss_active = state.ble_loss_enabled && !state.ble_connected;
        state.motors_blocked = state.collision_active ||
                               state.battery_block_active ||
                               state.line_loss_active ||
                               state.ble_loss_active;
        portEXIT_CRITICAL(&state_mux);

        vTaskDelay(pdMS_TO_TICKS(SAFETY_TASK_PERIOD_MS));
    }
}

esp_err_t safety_init(void)
{
    if (initialized) {
        return ESP_OK;
    }

    memset(&state, 0, sizeof(state));
    state.collision_enabled = true;
    state.battery_block_enabled = true;
    state.line_loss_enabled = true;
    state.ble_loss_enabled = true;
    state.ble_connected = false;
    state.roll_limit_deg = SAFETY_DEFAULT_ROLL_LIMIT_DEG;
    state.battery_block_percent = SAFETY_DEFAULT_BATTERY_BLOCK_PERCENT;
    state.line_loss_timeout_s = SAFETY_DEFAULT_LINE_LOSS_TIMEOUT_S;
    load_settings_from_nvs();
    state.ble_loss_active = state.ble_loss_enabled && !state.ble_connected;
    state.motors_blocked = state.ble_loss_active;

    BaseType_t created = xTaskCreatePinnedToCore(safety_task,
                                                 "safety",
                                                 3072,
                                                 NULL,
                                                 SAFETY_TASK_PRIORITY,
                                                 &safety_task_handle,
                                                 SAFETY_TASK_CORE_ID);
    if (created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    initialized = true;
    ESP_LOGI(TAG,
             "Seguranca iniciada roll=%.1fdeg bateria=%.1f%% linha=%.1fs ble=%d",
             state.roll_limit_deg,
             state.battery_block_percent,
             state.line_loss_timeout_s,
             state.ble_loss_enabled ? 1 : 0);
    return ESP_OK;
}

bool safety_motors_allowed(void)
{
    if (!initialized) {
        return true;
    }

    bool allowed = true;
    portENTER_CRITICAL(&state_mux);
    allowed = !state.motors_blocked;
    portEXIT_CRITICAL(&state_mux);
    return allowed;
}

bool safety_get_state(safety_state_t *out_state)
{
    if (out_state == NULL || !initialized) {
        return false;
    }

    portENTER_CRITICAL(&state_mux);
    *out_state = state;
    portEXIT_CRITICAL(&state_mux);
    return true;
}

esp_err_t safety_set_collision_enabled(bool enabled)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&state_mux);
    state.collision_enabled = enabled;
    portEXIT_CRITICAL(&state_mux);
    return save_settings_to_nvs();
}

esp_err_t safety_set_battery_block_enabled(bool enabled)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&state_mux);
    state.battery_block_enabled = enabled;
    portEXIT_CRITICAL(&state_mux);
    return save_settings_to_nvs();
}

esp_err_t safety_set_line_loss_enabled(bool enabled)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&state_mux);
    state.line_loss_enabled = enabled;
    if (!enabled) {
        state.line_loss_active = false;
    }
    portEXIT_CRITICAL(&state_mux);
    return save_settings_to_nvs();
}

esp_err_t safety_set_ble_loss_enabled(bool enabled)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&state_mux);
    state.ble_loss_enabled = enabled;
    if (!enabled) {
        state.ble_loss_active = false;
    }
    state.motors_blocked = state.collision_active ||
                           state.battery_block_active ||
                           state.line_loss_active ||
                           state.ble_loss_active;
    portEXIT_CRITICAL(&state_mux);
    return save_settings_to_nvs();
}

esp_err_t safety_set_ble_connected(bool connected)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&state_mux);
    state.ble_connected = connected;
    state.ble_loss_active = state.ble_loss_enabled && !state.ble_connected;
    state.motors_blocked = state.collision_active ||
                           state.battery_block_active ||
                           state.line_loss_active ||
                           state.ble_loss_active;
    portEXIT_CRITICAL(&state_mux);
    return ESP_OK;
}

esp_err_t safety_set_roll_limit_deg(float limit_deg)
{
    if (!initialized || !isfinite(limit_deg)) {
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&state_mux);
    state.roll_limit_deg = clamp_float(limit_deg, 1.0f, 90.0f);
    portEXIT_CRITICAL(&state_mux);
    return save_settings_to_nvs();
}

esp_err_t safety_set_line_loss_timeout_s(float timeout_s)
{
    if (!initialized || !isfinite(timeout_s)) {
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&state_mux);
    state.line_loss_timeout_s = clamp_float(timeout_s, 0.1f, 10.0f);
    portEXIT_CRITICAL(&state_mux);
    return save_settings_to_nvs();
}
esp_err_t safety_set_battery_block_percent(float percent)
{
    if (!initialized || !isfinite(percent)) {
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&state_mux);
    state.battery_block_percent = clamp_float(percent, 0.0f, 100.0f);
    portEXIT_CRITICAL(&state_mux);
    return save_settings_to_nvs();
}
