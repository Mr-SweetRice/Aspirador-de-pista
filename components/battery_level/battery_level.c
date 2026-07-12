#include "battery_level.h"

#include <string.h>

#include "battery_level_config.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BATTERY_LEVEL_CRITICAL_PERCENT 10.0f

static const char *TAG = "battery_level";

static adc_oneshot_unit_handle_t adc_handle;
static adc_channel_t adc_channel = BATTERY_LEVEL_ADC_CHANNEL;
static battery_level_state_t state;
static portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t battery_task_handle;
static bool initialized;

static float clamp_percent(float value)
{
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 100.0f) {
        return 100.0f;
    }
    return value;
}

static float raw_to_percent(uint16_t raw)
{
    const float span = (float)(BATTERY_LEVEL_RAW_MAX - BATTERY_LEVEL_RAW_MIN);
    if (span <= 0.0f) {
        return 0.0f;
    }
    return clamp_percent(((float)raw - (float)BATTERY_LEVEL_RAW_MIN) * 100.0f / span);
}

static float raw_to_voltage(uint16_t raw)
{
    const float voltage = ((float)raw * BATTERY_LEVEL_ADC_REFERENCE_V * BATTERY_LEVEL_VOLTAGE_SCALE) / 4095.0f;
    if (voltage < 0.0f) {
        return 0.0f;
    }
    if (voltage > BATTERY_LEVEL_VOLTAGE_MAX_V) {
        return BATTERY_LEVEL_VOLTAGE_MAX_V;
    }
    return voltage;
}

static void battery_task(void *arg)
{
    (void)arg;

    uint32_t raw_sum = 0;
    uint32_t samples = 0;
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t average_start = last_wake;
    TickType_t sample_period_ticks = pdMS_TO_TICKS(BATTERY_LEVEL_SAMPLE_PERIOD_MS);
    TickType_t average_period_ticks = pdMS_TO_TICKS(BATTERY_LEVEL_AVERAGE_PERIOD_MS);
    esp_err_t last_read_error = ESP_OK;
    bool first_sample_logged = false;

    if (sample_period_ticks < 1) {
        sample_period_ticks = 1;
    }
    if (average_period_ticks < 1) {
        average_period_ticks = 1;
    }

    while (true) {
        int raw = 0;
        esp_err_t ret = adc_oneshot_read(adc_handle, adc_channel, &raw);
        if (ret == ESP_OK) {
            const uint16_t raw_sample = (uint16_t)raw;
            raw_sum += (uint32_t)raw_sample;
            samples++;
            last_read_error = ESP_OK;

            if (!first_sample_logged) {
                ESP_LOGI(TAG,
                         "Primeira amostra bateria raw=%u voltage=%.2f percent=%.1f",
                         (unsigned int)raw_sample,
                         raw_to_voltage(raw_sample),
                         raw_to_percent(raw_sample));
                first_sample_logged = true;
            }
        } else if (ret != last_read_error) {
            ESP_LOGW(TAG, "Falha leitura ADC bateria gpio=%d channel=%d ret=%s",
                     BATTERY_LEVEL_ADC_GPIO,
                     adc_channel,
                     esp_err_to_name(ret));
            last_read_error = ret;
        }

        const TickType_t now = xTaskGetTickCount();
        if ((now - average_start) >= average_period_ticks) {
            average_start = now;
            if (samples == 0) {
                vTaskDelayUntil(&last_wake, sample_period_ticks);
                continue;
            }

            const uint16_t average_raw = (uint16_t)(raw_sum / samples);
            const float percent = raw_to_percent(average_raw);

            portENTER_CRITICAL(&state_mux);
            state.raw = average_raw;
            state.percent = percent;
            state.voltage_v = raw_to_voltage(average_raw);
            state.valid = true;
            portEXIT_CRITICAL(&state_mux);

            ESP_LOGI(TAG,
                     "Media bateria raw=%u voltage=%.2f percent=%.1f samples=%lu",
                     (unsigned int)average_raw,
                     raw_to_voltage(average_raw),
                     percent,
                     (unsigned long)samples);

            raw_sum = 0;
            samples = 0;
        }

        vTaskDelayUntil(&last_wake, sample_period_ticks);
    }
}

esp_err_t battery_level_init(void)
{
    if (initialized) {
        return ESP_OK;
    }

    adc_unit_t adc_unit = BATTERY_LEVEL_ADC_UNIT;
    adc_channel_t detected_channel = BATTERY_LEVEL_ADC_CHANNEL;
    esp_err_t io_ret = adc_oneshot_io_to_channel(BATTERY_LEVEL_ADC_GPIO, &adc_unit, &detected_channel);
    if (io_ret == ESP_OK) {
        adc_channel = detected_channel;
        if (adc_unit != BATTERY_LEVEL_ADC_UNIT || detected_channel != BATTERY_LEVEL_ADC_CHANNEL) {
            ESP_LOGW(TAG,
                     "GPIO bateria ajustou ADC gpio=%d unit=%d channel=%d config_unit=%d config_channel=%d",
                     BATTERY_LEVEL_ADC_GPIO,
                     adc_unit,
                     detected_channel,
                     BATTERY_LEVEL_ADC_UNIT,
                     BATTERY_LEVEL_ADC_CHANNEL);
        }
    } else {
        ESP_LOGW(TAG,
                 "Nao foi possivel mapear GPIO bateria=%d para ADC (%s), usando unit=%d channel=%d",
                 BATTERY_LEVEL_ADC_GPIO,
                 esp_err_to_name(io_ret),
                 BATTERY_LEVEL_ADC_UNIT,
                 BATTERY_LEVEL_ADC_CHANNEL);
    }

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = adc_unit,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&init_config, &adc_handle), TAG, "adc unit");

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = BATTERY_LEVEL_ADC_ATTEN,
        .bitwidth = BATTERY_LEVEL_ADC_BITWIDTH,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc_handle, adc_channel, &channel_config),
                        TAG,
                        "adc channel");

    memset(&state, 0, sizeof(state));

    BaseType_t created = xTaskCreatePinnedToCore(battery_task,
                                                 "battery_level",
                                                 BATTERY_LEVEL_TASK_STACK_SIZE,
                                                 NULL,
                                                 BATTERY_LEVEL_TASK_PRIORITY,
                                                 &battery_task_handle,
                                                 BATTERY_LEVEL_TASK_CORE_ID);
    if (created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    initialized = true;
    ESP_LOGI(TAG,
             "Bateria iniciada gpio=%d adc_unit=%d adc_channel=%d raw_min=%d raw_max=%d media=%dms core=%d priority=%d",
             BATTERY_LEVEL_ADC_GPIO,
             adc_unit,
             adc_channel,
             BATTERY_LEVEL_RAW_MIN,
             BATTERY_LEVEL_RAW_MAX,
             BATTERY_LEVEL_AVERAGE_PERIOD_MS,
             BATTERY_LEVEL_TASK_CORE_ID,
             BATTERY_LEVEL_TASK_PRIORITY);
    return ESP_OK;
}

bool battery_level_get_state(battery_level_state_t *out_state)
{
    if (out_state == NULL || !initialized) {
        return false;
    }

    portENTER_CRITICAL(&state_mux);
    *out_state = state;
    portEXIT_CRITICAL(&state_mux);
    return true;
}

bool battery_level_is_critical(void)
{
    battery_level_state_t local = {0};
    if (!battery_level_get_state(&local) || !local.valid) {
        return false;
    }
    return local.percent <= BATTERY_LEVEL_CRITICAL_PERCENT;
}
