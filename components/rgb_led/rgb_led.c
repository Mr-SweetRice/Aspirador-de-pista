#include "rgb_led.h"

#include <string.h>

#include "battery_level.h"
#include "driver/rmt_common.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "memory_config.h"
#include "nvs.h"
#include "rgb_led_config.h"
#include "safety.h"

#define RGB_LED_NVS_KEY "rgb_cfg"

typedef struct __attribute__((packed)) {
    uint8_t mode;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t intensity;
    uint8_t enabled;
} rgb_led_nvs_config_t;

static const char *TAG = "rgb_led";

static rmt_channel_handle_t tx_channel;
static rmt_encoder_handle_t bytes_encoder;
static rgb_led_state_t state = {
    .mode = RGB_LED_MODE_BATTERY,
    .red = 255,
    .green = 255,
    .blue = 255,
    .intensity = RGB_LED_DEFAULT_INTENSITY,
    .enabled = true,
};
static portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t rgb_led_task_handle;
static uint32_t race_plan_blink_period_ms;
static int64_t race_plan_blink_started_us;
static uint8_t intensity_before_race_plan = RGB_LED_DEFAULT_INTENSITY;
static bool race_plan_intensity_saved;
static bool initialized;

static void notify_rgb_led_task(void)
{
    TaskHandle_t task = NULL;
    portENTER_CRITICAL(&state_mux);
    task = rgb_led_task_handle;
    portEXIT_CRITICAL(&state_mux);

    if (task != NULL) {
        xTaskNotifyGive(task);
    }
}

static uint8_t scale_channel(uint8_t value, uint8_t intensity)
{
    return (uint8_t)(((uint16_t)value * (uint16_t)intensity) / 255U);
}

static void battery_color(float percent, uint8_t *red, uint8_t *green, uint8_t *blue)
{
    if (percent < 0.0f) {
        percent = 0.0f;
    } else if (percent > 100.0f) {
        percent = 100.0f;
    }

    *blue = 0;
    if (percent < 50.0f) {
        *red = 255;
        *green = (uint8_t)((percent / 50.0f) * 255.0f);
    } else {
        *red = (uint8_t)(((100.0f - percent) / 50.0f) * 255.0f);
        *green = 255;
    }
}

static float battery_alert_threshold_percent(void)
{
    safety_state_t safety = {0};
    if (safety_get_state(&safety)) {
        return safety.battery_block_percent;
    }
    return RGB_LED_BATTERY_ALERT_FALLBACK_PERCENT;
}

static bool battery_alert_active(const battery_level_state_t *battery)
{
    if (battery == NULL || !battery->valid) {
        return false;
    }

    const float threshold = battery_alert_threshold_percent();
    return threshold > 0.0f && battery->percent <= threshold;
}

static esp_err_t transmit_color(uint8_t red, uint8_t green, uint8_t blue, uint8_t intensity)
{
    uint8_t grb[3] = {
        scale_channel(green, intensity),
        scale_channel(red, intensity),
        scale_channel(blue, intensity),
    };
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    ESP_RETURN_ON_ERROR(rmt_transmit(tx_channel, bytes_encoder, grb, sizeof(grb), &tx_config), TAG, "rmt transmit");
    ESP_RETURN_ON_ERROR(rmt_tx_wait_all_done(tx_channel, 100), TAG, "rmt wait");
    esp_rom_delay_us(80);
    return ESP_OK;
}

static esp_err_t save_settings_to_nvs(void)
{
    rgb_led_state_t local;
    portENTER_CRITICAL(&state_mux);
    local = state;
    portEXIT_CRITICAL(&state_mux);

    rgb_led_nvs_config_t config = {
        .mode = (uint8_t)local.mode,
        .red = local.red,
        .green = local.green,
        .blue = local.blue,
        .intensity = local.intensity,
        .enabled = local.enabled ? 1 : 0,
    };

    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = nvs_set_blob(handle, RGB_LED_NVS_KEY, &config, sizeof(config));
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

    rgb_led_nvs_config_t config = {0};
    size_t len = sizeof(config);
    ret = nvs_get_blob(handle, RGB_LED_NVS_KEY, &config, &len);
    nvs_close(handle);
    if (ret != ESP_OK || len != sizeof(config) || config.mode > RGB_LED_MODE_RACE_PLAN) {
        return;
    }

    portENTER_CRITICAL(&state_mux);
    state.mode = (rgb_led_mode_t)config.mode;
    state.red = config.red;
    state.green = config.green;
    state.blue = config.blue;
    state.intensity = config.intensity;
    state.enabled = config.enabled != 0;
    portEXIT_CRITICAL(&state_mux);
}

static void rgb_led_task(void *arg)
{
    (void)arg;

    while (true) {
        rgb_led_state_t local;
        uint32_t local_blink_period_ms = 0;
        int64_t local_blink_started_us = 0;
        portENTER_CRITICAL(&state_mux);
        local = state;
        local_blink_period_ms = race_plan_blink_period_ms;
        local_blink_started_us = race_plan_blink_started_us;
        portEXIT_CRITICAL(&state_mux);

        if (!local.enabled || local.mode == RGB_LED_MODE_DISABLED) {
            transmit_color(0, 0, 0, 0);
        } else {
            battery_level_state_t battery = {0};
            uint8_t red = 255;
            uint8_t green = 0;
            uint8_t blue = 0;

            if (battery_level_get_state(&battery) && battery_alert_active(&battery)) {
                const bool blink_on = ((esp_timer_get_time() / 500000) % 2) == 0;
                transmit_color(blink_on ? 255 : 0, 0, 0, local.intensity);
            } else if (local.mode == RGB_LED_MODE_BATTERY) {
                if (battery.valid) {
                    battery_color(battery.percent, &red, &green, &blue);
                }
                transmit_color(red, green, blue, local.intensity);
            } else {
                bool blink_on = true;
                if (local.mode == RGB_LED_MODE_RACE_PLAN && local_blink_period_ms > 0) {
                    const int64_t half_period_us = ((int64_t)local_blink_period_ms * 1000LL) / 2LL;
                    const int64_t elapsed_us = esp_timer_get_time() - local_blink_started_us;
                    blink_on = half_period_us <= 0 || ((elapsed_us / half_period_us) % 2LL) == 0;
                }
                transmit_color(blink_on ? local.red : 0,
                               blink_on ? local.green : 0,
                               blink_on ? local.blue : 0,
                               local.intensity);
            }
        }

        uint32_t wait_ms = RGB_LED_UPDATE_PERIOD_MS;
        if (local.mode == RGB_LED_MODE_RACE_PLAN && local_blink_period_ms > 0) {
            const uint32_t half_period_ms = local_blink_period_ms > 1 ? local_blink_period_ms / 2U : 1U;
            if (half_period_ms < wait_ms) {
                wait_ms = half_period_ms;
            }
        }
        TickType_t wait_ticks = pdMS_TO_TICKS(wait_ms);
        if (wait_ticks == 0) {
            wait_ticks = 1;
        }
        ulTaskNotifyTake(pdTRUE, wait_ticks);
    }
}

esp_err_t rgb_led_init(void)
{
    if (initialized) {
        return ESP_OK;
    }

    rmt_tx_channel_config_t tx_channel_config = {
        .gpio_num = RGB_LED_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RGB_LED_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 1,
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_channel_config, &tx_channel), TAG, "rmt tx");

    rmt_bytes_encoder_config_t encoder_config = {
        .bit0 = {
            .duration0 = 4,
            .level0 = 1,
            .duration1 = 8,
            .level1 = 0,
        },
        .bit1 = {
            .duration0 = 8,
            .level0 = 1,
            .duration1 = 4,
            .level1 = 0,
        },
        .flags = {
            .msb_first = 1,
        },
    };
    ESP_RETURN_ON_ERROR(rmt_new_bytes_encoder(&encoder_config, &bytes_encoder), TAG, "rmt encoder");
    ESP_RETURN_ON_ERROR(rmt_enable(tx_channel), TAG, "rmt enable");

    load_settings_from_nvs();

    BaseType_t created = xTaskCreatePinnedToCore(rgb_led_task,
                                                 "rgb_led",
                                                 3072,
                                                 NULL,
                                                 RGB_LED_TASK_PRIORITY,
                                                 &rgb_led_task_handle,
                                                 RGB_LED_TASK_CORE_ID);
    if (created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    initialized = true;
    ESP_LOGI(TAG, "RGB LED iniciado gpio=%d core=%d", RGB_LED_GPIO, RGB_LED_TASK_CORE_ID);
    return ESP_OK;
}

esp_err_t rgb_led_set_enabled(bool enabled)
{
    bool changed = false;
    portENTER_CRITICAL(&state_mux);
    changed = state.enabled != enabled;
    state.enabled = enabled;
    portEXIT_CRITICAL(&state_mux);
    if (changed) {
        notify_rgb_led_task();
    }
    return save_settings_to_nvs();
}

esp_err_t rgb_led_set_mode(rgb_led_mode_t mode)
{
    if (mode != RGB_LED_MODE_DISABLED &&
        mode != RGB_LED_MODE_BATTERY &&
        mode != RGB_LED_MODE_MANUAL &&
        mode != RGB_LED_MODE_RACE_PLAN) {
        return ESP_ERR_INVALID_ARG;
    }
    bool changed = false;
    portENTER_CRITICAL(&state_mux);
    if (mode != RGB_LED_MODE_RACE_PLAN && race_plan_intensity_saved) {
        state.intensity = intensity_before_race_plan;
        race_plan_intensity_saved = false;
    } else if (mode == RGB_LED_MODE_RACE_PLAN && !race_plan_intensity_saved) {
        intensity_before_race_plan = state.mode == RGB_LED_MODE_RACE_PLAN ?
                                         RGB_LED_DEFAULT_INTENSITY :
                                         state.intensity;
        race_plan_intensity_saved = true;
        state.intensity = RGB_LED_RACE_PLAN_INTENSITY;
    }
    changed = state.mode != mode || state.enabled != (mode != RGB_LED_MODE_DISABLED) ||
              race_plan_blink_period_ms != 0;
    state.mode = mode;
    state.enabled = mode != RGB_LED_MODE_DISABLED;
    race_plan_blink_period_ms = 0;
    portEXIT_CRITICAL(&state_mux);
    if (changed) {
        notify_rgb_led_task();
    }
    return save_settings_to_nvs();
}

esp_err_t rgb_led_set_manual_color(uint8_t red, uint8_t green, uint8_t blue, uint8_t intensity)
{
    bool changed = false;
    portENTER_CRITICAL(&state_mux);
    race_plan_intensity_saved = false;
    changed = state.red != red || state.green != green || state.blue != blue ||
              state.intensity != intensity || state.mode != RGB_LED_MODE_MANUAL ||
              !state.enabled || race_plan_blink_period_ms != 0;
    state.red = red;
    state.green = green;
    state.blue = blue;
    state.intensity = intensity;
    state.mode = RGB_LED_MODE_MANUAL;
    state.enabled = true;
    race_plan_blink_period_ms = 0;
    portEXIT_CRITICAL(&state_mux);
    if (changed) {
        notify_rgb_led_task();
    }
    return save_settings_to_nvs();
}

esp_err_t rgb_led_set_race_plan_color(uint8_t red, uint8_t green, uint8_t blue)
{
    bool changed = false;
    portENTER_CRITICAL(&state_mux);
    if (!race_plan_intensity_saved) {
        intensity_before_race_plan = state.mode == RGB_LED_MODE_RACE_PLAN ?
                                         RGB_LED_DEFAULT_INTENSITY :
                                         state.intensity;
        race_plan_intensity_saved = true;
    }
    changed = state.red != red || state.green != green || state.blue != blue ||
              state.mode != RGB_LED_MODE_RACE_PLAN || !state.enabled ||
              state.intensity != RGB_LED_RACE_PLAN_INTENSITY || race_plan_blink_period_ms != 0;
    state.red = red;
    state.green = green;
    state.blue = blue;
    state.intensity = RGB_LED_RACE_PLAN_INTENSITY;
    state.mode = RGB_LED_MODE_RACE_PLAN;
    state.enabled = true;
    race_plan_blink_period_ms = 0;
    portEXIT_CRITICAL(&state_mux);

    if (changed) {
        notify_rgb_led_task();
    }
    return ESP_OK;
}

esp_err_t rgb_led_set_battery_mode_runtime(void)
{
    bool changed = false;
    portENTER_CRITICAL(&state_mux);
    changed = state.mode != RGB_LED_MODE_BATTERY || !state.enabled ||
              race_plan_blink_period_ms != 0;
    if (race_plan_intensity_saved) {
        state.intensity = intensity_before_race_plan;
        race_plan_intensity_saved = false;
    }
    state.mode = RGB_LED_MODE_BATTERY;
    state.enabled = true;
    race_plan_blink_period_ms = 0;
    portEXIT_CRITICAL(&state_mux);

    if (changed) {
        notify_rgb_led_task();
    }
    return ESP_OK;
}

esp_err_t rgb_led_set_race_plan_blink(uint8_t red,
                                      uint8_t green,
                                      uint8_t blue,
                                      uint32_t period_ms)
{
    if (period_ms == 0) {
        return rgb_led_set_race_plan_color(red, green, blue);
    }

    const int64_t now_us = esp_timer_get_time();
    bool changed = false;
    portENTER_CRITICAL(&state_mux);
    if (!race_plan_intensity_saved) {
        intensity_before_race_plan = state.mode == RGB_LED_MODE_RACE_PLAN ?
                                         RGB_LED_DEFAULT_INTENSITY :
                                         state.intensity;
        race_plan_intensity_saved = true;
    }
    changed = state.red != red || state.green != green || state.blue != blue ||
              state.mode != RGB_LED_MODE_RACE_PLAN || !state.enabled ||
              state.intensity != RGB_LED_RACE_PLAN_INTENSITY || race_plan_blink_period_ms != period_ms;
    state.red = red;
    state.green = green;
    state.blue = blue;
    state.intensity = RGB_LED_RACE_PLAN_INTENSITY;
    state.mode = RGB_LED_MODE_RACE_PLAN;
    state.enabled = true;
    race_plan_blink_period_ms = period_ms;
    if (changed) {
        race_plan_blink_started_us = now_us;
    }
    portEXIT_CRITICAL(&state_mux);

    if (changed) {
        notify_rgb_led_task();
    }
    return ESP_OK;
}

bool rgb_led_get_state(rgb_led_state_t *out_state)
{
    if (out_state == NULL || !initialized) {
        return false;
    }
    portENTER_CRITICAL(&state_mux);
    *out_state = state;
    portEXIT_CRITICAL(&state_mux);
    return true;
}
