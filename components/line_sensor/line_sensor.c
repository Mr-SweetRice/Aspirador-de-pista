#include "line_sensor.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "memory_config.h"
#include "nvs.h"
#include "soc/gpio_struct.h"

#define LINE_SENSOR_NVS_KEY_MIN_BLACK "qtr_min_b"
#define LINE_SENSOR_NVS_KEY_MAX_BLACK "qtr_max_b"
#define LINE_SENSOR_NVS_KEY_MIN_WHITE "qtr_min_w"
#define LINE_SENSOR_NVS_KEY_MAX_WHITE "qtr_max_w"
#define LINE_SENSOR_NVS_KEY_MIN_LEGACY "qtr_min"
#define LINE_SENSOR_NVS_KEY_MAX_LEGACY "qtr_max"
#define LINE_SENSOR_NVS_KEY_TRACK "qtr_track"
#define LINE_SENSOR_NVS_KEY_THRESHOLD "qtr_thr"
#define LINE_SENSOR_THRESHOLD_PERCENT_DEFAULT 0
#define LINE_SENSOR_THRESHOLD_PERCENT_MAX 45

static const char *TAG = "line_sensor";

static const gpio_num_t sensor_gpios[LINE_SENSOR_QTR_COUNT] = {
    LINE_SENSOR_QTR_1_GPIO,
    LINE_SENSOR_QTR_2_GPIO,
    LINE_SENSOR_QTR_3_GPIO,
    LINE_SENSOR_QTR_4_GPIO,
    LINE_SENSOR_QTR_5_GPIO,
    LINE_SENSOR_QTR_6_GPIO,
    LINE_SENSOR_QTR_7_GPIO,
    LINE_SENSOR_QTR_8_GPIO,
};

static const uint32_t sensor_gpio_mask = (1UL << LINE_SENSOR_QTR_1_GPIO) |
                                         (1UL << LINE_SENSOR_QTR_2_GPIO) |
                                         (1UL << LINE_SENSOR_QTR_3_GPIO) |
                                         (1UL << LINE_SENSOR_QTR_4_GPIO) |
                                         (1UL << LINE_SENSOR_QTR_5_GPIO) |
                                         (1UL << LINE_SENSOR_QTR_6_GPIO) |
                                         (1UL << LINE_SENSOR_QTR_7_GPIO) |
                                         (1UL << LINE_SENSOR_QTR_8_GPIO);

static line_sensor_state_t state;
static uint16_t calibration_min[2][LINE_SENSOR_QTR_COUNT];
static uint16_t calibration_max[2][LINE_SENSOR_QTR_COUNT];
static bool calibration_valid[2];
static line_sensor_track_type_t track_type = LINE_SENSOR_TRACK_BLACK;
static uint8_t threshold_percent = LINE_SENSOR_THRESHOLD_PERCENT_DEFAULT;
static portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;
static int64_t calibration_end_us;
static bool calibrating;
static bool initialized;
static TaskHandle_t line_sensor_task_handle;
static esp_timer_handle_t line_sensor_timer_handle;
static uint16_t filtered_values[LINE_SENSOR_QTR_COUNT];
static uint16_t filtered_position;
static bool have_filtered_sample;
static uint32_t sample_count;
static int64_t sample_window_start_us;
static float current_read_hz;

static void line_sensor_timer_cb(void *arg)
{
    (void)arg;
    if (line_sensor_task_handle != NULL) {
        xTaskNotifyGive(line_sensor_task_handle);
    }
}

static void line_sensor_task(void *arg)
{
    (void)arg;
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        (void)line_sensor_sample_now();
    }
}

static int logical_index_from_physical(int physical_index)
{
#if LINE_SENSOR_REVERSE_ORDER
    return (LINE_SENSOR_QTR_COUNT - 1) - physical_index;
#else
    return physical_index;
#endif
}

static void clamp_calibration_to_current_timeout(void)
{
    for (int track = 0; track <= LINE_SENSOR_TRACK_WHITE; ++track) {
        for (int i = 0; i < LINE_SENSOR_QTR_COUNT; ++i) {
            if (calibration_min[track][i] > LINE_SENSOR_QTR_TIMEOUT_US) {
                calibration_min[track][i] = LINE_SENSOR_QTR_TIMEOUT_US;
            }
            if (calibration_max[track][i] > LINE_SENSOR_QTR_TIMEOUT_US) {
                calibration_max[track][i] = LINE_SENSOR_QTR_TIMEOUT_US;
            }
        }
    }
}

static esp_err_t save_blob_to_nvs(const char *key, const void *value, size_t len)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = nvs_set_blob(handle, key, value, len);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    return ret;
}

static void load_settings_from_nvs(void)
{
    for (int track = 0; track <= LINE_SENSOR_TRACK_WHITE; ++track) {
        for (int i = 0; i < LINE_SENSOR_QTR_COUNT; ++i) {
            calibration_min[track][i] = LINE_SENSOR_QTR_TIMEOUT_US;
            calibration_max[track][i] = 0;
        }
        calibration_valid[track] = false;
    }

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        return;
    }

    const char *min_keys[2] = {LINE_SENSOR_NVS_KEY_MIN_BLACK, LINE_SENSOR_NVS_KEY_MIN_WHITE};
    const char *max_keys[2] = {LINE_SENSOR_NVS_KEY_MAX_BLACK, LINE_SENSOR_NVS_KEY_MAX_WHITE};
    for (int track = 0; track <= LINE_SENSOR_TRACK_WHITE; ++track) {
        size_t len = sizeof(calibration_min[track]);
        bool valid = nvs_get_blob(handle, min_keys[track], calibration_min[track], &len) == ESP_OK &&
                     len == sizeof(calibration_min[track]);
        len = sizeof(calibration_max[track]);
        valid = valid &&
                nvs_get_blob(handle, max_keys[track], calibration_max[track], &len) == ESP_OK &&
                len == sizeof(calibration_max[track]);
        calibration_valid[track] = valid;
    }

    if (!calibration_valid[LINE_SENSOR_TRACK_BLACK] && !calibration_valid[LINE_SENSOR_TRACK_WHITE]) {
        size_t len = sizeof(calibration_min[LINE_SENSOR_TRACK_BLACK]);
        bool legacy_valid = nvs_get_blob(handle,
                                         LINE_SENSOR_NVS_KEY_MIN_LEGACY,
                                         calibration_min[LINE_SENSOR_TRACK_BLACK],
                                         &len) == ESP_OK &&
                            len == sizeof(calibration_min[LINE_SENSOR_TRACK_BLACK]);
        len = sizeof(calibration_max[LINE_SENSOR_TRACK_BLACK]);
        legacy_valid = legacy_valid &&
                       nvs_get_blob(handle,
                                    LINE_SENSOR_NVS_KEY_MAX_LEGACY,
                                    calibration_max[LINE_SENSOR_TRACK_BLACK],
                                    &len) == ESP_OK &&
                       len == sizeof(calibration_max[LINE_SENSOR_TRACK_BLACK]);
        if (legacy_valid) {
            memcpy(calibration_min[LINE_SENSOR_TRACK_WHITE],
                   calibration_min[LINE_SENSOR_TRACK_BLACK],
                   sizeof(calibration_min[LINE_SENSOR_TRACK_WHITE]));
            memcpy(calibration_max[LINE_SENSOR_TRACK_WHITE],
                   calibration_max[LINE_SENSOR_TRACK_BLACK],
                   sizeof(calibration_max[LINE_SENSOR_TRACK_WHITE]));
            calibration_valid[LINE_SENSOR_TRACK_BLACK] = true;
            calibration_valid[LINE_SENSOR_TRACK_WHITE] = true;
        }
    }

    uint8_t stored_track = (uint8_t)track_type;
    ret = nvs_get_u8(handle, LINE_SENSOR_NVS_KEY_TRACK, &stored_track);
    if (ret == ESP_OK && stored_track <= LINE_SENSOR_TRACK_WHITE) {
        track_type = (line_sensor_track_type_t)stored_track;
    }
    uint8_t stored_threshold = threshold_percent;
    ret = nvs_get_u8(handle, LINE_SENSOR_NVS_KEY_THRESHOLD, &stored_threshold);
    if (ret == ESP_OK && stored_threshold <= LINE_SENSOR_THRESHOLD_PERCENT_MAX) {
        threshold_percent = stored_threshold;
    }
    nvs_close(handle);
    clamp_calibration_to_current_timeout();

    portENTER_CRITICAL(&state_mux);
    state.calibrated_valid = calibration_valid[track_type];
    state.threshold_percent = threshold_percent;
    portEXIT_CRITICAL(&state_mux);
}

static esp_err_t save_calibration_to_nvs(line_sensor_track_type_t track)
{
    const char *min_key = track == LINE_SENSOR_TRACK_WHITE ? LINE_SENSOR_NVS_KEY_MIN_WHITE : LINE_SENSOR_NVS_KEY_MIN_BLACK;
    const char *max_key = track == LINE_SENSOR_TRACK_WHITE ? LINE_SENSOR_NVS_KEY_MAX_WHITE : LINE_SENSOR_NVS_KEY_MAX_BLACK;
    esp_err_t ret = save_blob_to_nvs(min_key, calibration_min[track], sizeof(calibration_min[track]));
    if (ret == ESP_OK) {
        ret = save_blob_to_nvs(max_key, calibration_max[track], sizeof(calibration_max[track]));
    }
    return ret;
}

static void read_raw(uint16_t out_raw[LINE_SENSOR_QTR_COUNT])
{
    uint32_t active_mask = sensor_gpio_mask;

    for (int i = 0; i < LINE_SENSOR_QTR_COUNT; ++i) {
        out_raw[i] = LINE_SENSOR_QTR_TIMEOUT_US;
    }

    GPIO.out_w1ts = sensor_gpio_mask;
    GPIO.enable_w1ts = sensor_gpio_mask;
    esp_rom_delay_us(LINE_SENSOR_QTR_CHARGE_US);
    GPIO.enable_w1tc = sensor_gpio_mask;

    const int64_t start_us = esp_timer_get_time();
    while (active_mask != 0) {
        const uint16_t elapsed_us = (uint16_t)(esp_timer_get_time() - start_us);
        if (elapsed_us >= LINE_SENSOR_QTR_TIMEOUT_US) {
            break;
        }
        const uint32_t low_mask = active_mask & ~GPIO.in;
        if (low_mask == 0) {
            continue;
        }
        for (int i = 0; i < LINE_SENSOR_QTR_COUNT; ++i) {
            const uint32_t bit = 1UL << sensor_gpios[i];
            if (low_mask & bit) {
                out_raw[i] = elapsed_us;
                active_mask &= ~bit;
            }
        }
    }
}

static void read_raw_averaged(uint16_t out_raw[LINE_SENSOR_QTR_COUNT])
{
    uint32_t sums[LINE_SENSOR_QTR_COUNT] = {0};
    uint16_t sample[LINE_SENSOR_QTR_COUNT] = {0};

    for (int sample_index = 0; sample_index < LINE_SENSOR_RAW_SAMPLES; ++sample_index) {
        read_raw(sample);
        for (int i = 0; i < LINE_SENSOR_QTR_COUNT; ++i) {
            sums[i] += sample[i];
        }
    }

    for (int i = 0; i < LINE_SENSOR_QTR_COUNT; ++i) {
        out_raw[i] = (uint16_t)((sums[i] + (LINE_SENSOR_RAW_SAMPLES / 2U)) / LINE_SENSOR_RAW_SAMPLES);
    }
}

static uint16_t calibrate_one(uint16_t raw, uint16_t min_value, uint16_t max_value)
{
    if (max_value <= min_value + 1) {
        return 0;
    }
    const uint32_t range = (uint32_t)(max_value - min_value);
    const uint32_t margin = (range * threshold_percent) / 100U;
    const uint16_t effective_min = (uint16_t)(min_value + margin);
    const uint16_t effective_max = (uint16_t)(max_value - margin);
    if (effective_max <= effective_min + 1) {
        return 0;
    }
    if (raw <= effective_min) {
        return 0;
    }
    if (raw >= effective_max) {
        return 1000;
    }
    return (uint16_t)(((uint32_t)(raw - effective_min) * 1000U) / (uint32_t)(effective_max - effective_min));
}

static uint16_t lowpass_u16(uint16_t previous, uint16_t next, uint8_t alpha_q8)
{
    const uint32_t alpha = alpha_q8;
    const uint32_t filtered = ((uint32_t)previous * (256U - alpha)) + ((uint32_t)next * alpha) + 128U;
    return (uint16_t)(filtered >> 8);
}

static uint16_t lowpass_position(uint16_t previous, uint16_t next)
{
    int32_t delta = (int32_t)next - (int32_t)previous;
    const int32_t filtered = (int32_t)previous +
                             (int32_t)((delta * LINE_SENSOR_POSITION_FILTER_ALPHA_Q8) / 256);
    if (filtered <= 0) {
        return 0;
    }
    if (filtered >= 7000) {
        return 7000;
    }
    return (uint16_t)filtered;
}

static void filter_line_values(const uint16_t input[LINE_SENSOR_QTR_COUNT],
                               uint16_t output[LINE_SENSOR_QTR_COUNT])
{
    if (!have_filtered_sample || calibrating) {
        memcpy(filtered_values, input, sizeof(filtered_values));
        memcpy(output, input, sizeof(filtered_values));
        return;
    }

    for (int i = 0; i < LINE_SENSOR_QTR_COUNT; ++i) {
        filtered_values[i] = lowpass_u16(filtered_values[i],
                                         input[i],
                                         LINE_SENSOR_VALUE_FILTER_ALPHA_Q8);
        output[i] = filtered_values[i];
    }
}

static uint16_t estimate_position(const uint16_t values[LINE_SENSOR_QTR_COUNT], bool *line_visible)
{
    uint32_t weighted_sum = 0;
    uint32_t sum = 0;
    bool visible = false;

    for (int i = 0; i < LINE_SENSOR_QTR_COUNT; ++i) {
        const uint16_t value = values[i];
        if (value > LINE_SENSOR_POSITION_VISIBLE_THRESHOLD) {
            visible = true;
        }
        if (value > LINE_SENSOR_POSITION_VALUE_FLOOR) {
            weighted_sum += (uint32_t)value * (uint32_t)i * 1000U;
            sum += value;
        }
    }

    *line_visible = visible;
    if (sum == 0) {
        return state.position;
    }
    return (uint16_t)(weighted_sum / sum);
}

static uint16_t filter_position(uint16_t position, bool line_visible)
{
    if (!line_visible || !have_filtered_sample || calibrating) {
        filtered_position = position;
        have_filtered_sample = true;
        return position;
    }

    filtered_position = lowpass_position(filtered_position, position);
    return filtered_position;
}

static void update_calibration(const uint16_t raw[LINE_SENSOR_QTR_COUNT])
{
    for (int i = 0; i < LINE_SENSOR_QTR_COUNT; ++i) {
        if (raw[i] < calibration_min[track_type][i]) {
            calibration_min[track_type][i] = raw[i];
        }
        if (raw[i] > calibration_max[track_type][i]) {
            calibration_max[track_type][i] = raw[i];
        }
    }
}

static void finish_calibration_if_needed(void)
{
    if (!calibrating || esp_timer_get_time() <= calibration_end_us) {
        return;
    }

    calibrating = false;
    const line_sensor_track_type_t calibrated_track = track_type;
    esp_err_t ret = save_calibration_to_nvs(calibrated_track);
    if (ret == ESP_OK) {
        calibration_valid[calibrated_track] = true;
    }

    portENTER_CRITICAL(&state_mux);
    state.calibrating = false;
    state.calibrated_valid = calibration_valid[track_type];
    portEXIT_CRITICAL(&state_mux);

    ESP_LOGI(TAG,
             "Calibracao QTR finalizada track=%s ret=%s",
             calibrated_track == LINE_SENSOR_TRACK_BLACK ? "preta" : "branca",
             esp_err_to_name(ret));
}

esp_err_t line_sensor_sample_now(void)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const int64_t now_us = esp_timer_get_time();
    ++sample_count;
    const int64_t window_us = now_us - sample_window_start_us;
    if (window_us >= 1000000) {
        current_read_hz = ((float)sample_count * 1000000.0f) / (float)window_us;
        sample_count = 0;
        sample_window_start_us = now_us;
    }

    uint16_t raw[LINE_SENSOR_QTR_COUNT] = {0};
    uint16_t raw_logical[LINE_SENSOR_QTR_COUNT] = {0};
    uint16_t calibrated[LINE_SENSOR_QTR_COUNT] = {0};
    uint16_t calibrated_logical[LINE_SENSOR_QTR_COUNT] = {0};
    uint16_t values[LINE_SENSOR_QTR_COUNT] = {0};
    uint16_t stable_values[LINE_SENSOR_QTR_COUNT] = {0};

    read_raw_averaged(raw);
    if (calibrating) {
        update_calibration(raw);
    }
    finish_calibration_if_needed();

    bool line_visible = false;
    for (int i = 0; i < LINE_SENSOR_QTR_COUNT; ++i) {
        const int logical = logical_index_from_physical(i);
        calibrated[i] = calibrate_one(raw[i], calibration_min[track_type][i], calibration_max[track_type][i]);
        raw_logical[logical] = raw[i];
        calibrated_logical[logical] = calibrated[i];
        values[logical] = track_type == LINE_SENSOR_TRACK_BLACK ? calibrated[i] : (uint16_t)(1000U - calibrated[i]);
    }
    filter_line_values(values, stable_values);

    uint16_t position = estimate_position(stable_values, &line_visible);
    position = filter_position(position, line_visible);

    portENTER_CRITICAL(&state_mux);
    memcpy(state.raw, raw_logical, sizeof(state.raw));
    memcpy(state.calibrated, calibrated_logical, sizeof(state.calibrated));
    memcpy(state.line_values, stable_values, sizeof(state.line_values));
    state.position = position;
    state.line_visible = line_visible;
    state.calibrating = calibrating;
    state.track_type = track_type;
    state.calibrated_valid = calibration_valid[track_type];
    state.threshold_percent = threshold_percent;
    state.read_hz = current_read_hz;
    portEXIT_CRITICAL(&state_mux);

    return ESP_OK;
}

esp_err_t line_sensor_init(void)
{
    if (initialized) {
        return ESP_OK;
    }

    memset(&state, 0, sizeof(state));
    memset(filtered_values, 0, sizeof(filtered_values));
    filtered_position = 0;
    have_filtered_sample = false;
    sample_count = 0;
    sample_window_start_us = esp_timer_get_time();
    current_read_hz = 0.0f;
    load_settings_from_nvs();

    for (int i = 0; i < LINE_SENSOR_QTR_COUNT; ++i) {
        ESP_RETURN_ON_ERROR(gpio_reset_pin(sensor_gpios[i]), TAG, "qtr gpio reset");
        ESP_RETURN_ON_ERROR(gpio_set_pull_mode(sensor_gpios[i], GPIO_FLOATING), TAG, "qtr gpio floating");
        ESP_RETURN_ON_ERROR(gpio_set_direction(sensor_gpios[i], GPIO_MODE_INPUT), TAG, "qtr gpio input");
    }

    initialized = true;
    ESP_RETURN_ON_ERROR(line_sensor_sample_now(), TAG, "qtr first sample");

    if (line_sensor_task_handle == NULL) {
        BaseType_t created = xTaskCreatePinnedToCore(line_sensor_task,
                                                     "line_sensor",
                                                     4096,
                                                     NULL,
                                                     LINE_SENSOR_TASK_PRIORITY,
                                                     &line_sensor_task_handle,
                                                     LINE_SENSOR_TASK_CORE_ID);
        if (created != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (line_sensor_timer_handle == NULL) {
        const esp_timer_create_args_t timer_args = {
            .callback = line_sensor_timer_cb,
            .skip_unhandled_events = true,
            .name = "line_sensor",
        };
        esp_err_t timer_ret = esp_timer_create(&timer_args, &line_sensor_timer_handle);
        if (timer_ret != ESP_OK) {
            return timer_ret;
        }
        timer_ret = esp_timer_start_periodic(line_sensor_timer_handle, LINE_SENSOR_READ_PERIOD_US);
        if (timer_ret != ESP_OK) {
            return timer_ret;
        }
    }

    ESP_LOGI(TAG,
             "QRE/QTR iniciado pins=%d,%d,%d,%d,%d,%d,%d,%d target=%uHz period=%dus timeout=%dus samples=%d core=%d priority=%d",
             sensor_gpios[0],
             sensor_gpios[1],
             sensor_gpios[2],
             sensor_gpios[3],
             sensor_gpios[4],
             sensor_gpios[5],
             sensor_gpios[6],
             sensor_gpios[7],
             (unsigned int)LINE_SENSOR_READ_TARGET_HZ,
             LINE_SENSOR_READ_PERIOD_US,
             LINE_SENSOR_QTR_TIMEOUT_US,
             LINE_SENSOR_RAW_SAMPLES,
             LINE_SENSOR_TASK_CORE_ID,
             LINE_SENSOR_TASK_PRIORITY);
    return ESP_OK;
}

esp_err_t line_sensor_start_calibration(uint32_t duration_ms)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (duration_ms == 0) {
        duration_ms = LINE_SENSOR_CALIBRATION_DEFAULT_MS;
    }

    for (int i = 0; i < LINE_SENSOR_QTR_COUNT; ++i) {
        calibration_min[track_type][i] = LINE_SENSOR_QTR_TIMEOUT_US;
        calibration_max[track_type][i] = 0;
    }
    calibration_valid[track_type] = false;
    calibration_end_us = esp_timer_get_time() + ((int64_t)duration_ms * 1000);
    calibrating = true;
    have_filtered_sample = false;

    portENTER_CRITICAL(&state_mux);
    state.calibrating = true;
    state.calibrated_valid = false;
    state.threshold_percent = threshold_percent;
    portEXIT_CRITICAL(&state_mux);

    ESP_LOGI(TAG,
             "Calibracao QTR iniciada por %lu ms track=%s",
             (unsigned long)duration_ms,
             track_type == LINE_SENSOR_TRACK_BLACK ? "preta" : "branca");
    return ESP_OK;
}

esp_err_t line_sensor_set_track_type(line_sensor_track_type_t next_track_type)
{
    if (next_track_type != LINE_SENSOR_TRACK_BLACK && next_track_type != LINE_SENSOR_TRACK_WHITE) {
        return ESP_ERR_INVALID_ARG;
    }

    track_type = next_track_type;
    portENTER_CRITICAL(&state_mux);
    state.track_type = track_type;
    state.calibrated_valid = calibration_valid[track_type];
    portEXIT_CRITICAL(&state_mux);

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = nvs_set_u8(handle, LINE_SENSOR_NVS_KEY_TRACK, (uint8_t)track_type);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);

    ESP_LOGI(TAG, "Tipo de pista QTR=%s", track_type == LINE_SENSOR_TRACK_BLACK ? "linha preta" : "linha branca");
    return ret;
}

esp_err_t line_sensor_set_threshold_percent(uint8_t next_threshold_percent)
{
    if (next_threshold_percent > LINE_SENSOR_THRESHOLD_PERCENT_MAX) {
        next_threshold_percent = LINE_SENSOR_THRESHOLD_PERCENT_MAX;
    }

    threshold_percent = next_threshold_percent;
    portENTER_CRITICAL(&state_mux);
    state.threshold_percent = threshold_percent;
    portEXIT_CRITICAL(&state_mux);

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = nvs_set_u8(handle, LINE_SENSOR_NVS_KEY_THRESHOLD, threshold_percent);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);

    ESP_LOGI(TAG, "Limiar QTR=%u%%", (unsigned int)threshold_percent);
    return ret;
}

line_sensor_track_type_t line_sensor_get_track_type(void)
{
    return track_type;
}

bool line_sensor_get_state(line_sensor_state_t *out_state)
{
    if (out_state == NULL || !initialized) {
        return false;
    }

    portENTER_CRITICAL(&state_mux);
    *out_state = state;
    portEXIT_CRITICAL(&state_mux);
    return true;
}
