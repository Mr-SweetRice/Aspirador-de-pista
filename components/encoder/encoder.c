#include "encoder.h"

#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "encoder";

typedef struct {
    pcnt_unit_handle_t unit;
    pcnt_channel_handle_t channel_a;
    pcnt_channel_handle_t channel_b;
} encoder_runtime_t;

static encoder_runtime_t encoders[ENCODER_COUNT];
static encoder_state_t state;
static portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;
static int64_t last_sample_us;
static bool initialized;

static int apply_invert(int value, int invert)
{
    return invert ? -value : value;
}

static esp_err_t configure_encoder_gpio(gpio_num_t gpio)
{
    gpio_config_t io_config = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_config), TAG, "encoder gpio input");
    ESP_RETURN_ON_ERROR(gpio_set_pull_mode(gpio, GPIO_FLOATING), TAG, "encoder floating");
    return ESP_OK;
}

static esp_err_t configure_encoder_unit(const encoder_channel_config_t *config, encoder_runtime_t *runtime)
{
    pcnt_unit_config_t unit_config = {
        .low_limit = -30000,
        .high_limit = 30000,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_config, &runtime->unit), TAG, "pcnt unit");

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = ENCODER_PCNT_GLITCH_FILTER_NS,
    };
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(runtime->unit, &filter_config), TAG, "pcnt glitch");

    pcnt_chan_config_t channel_a_config = {
        .edge_gpio_num = config->channel_a_gpio,
        .level_gpio_num = config->channel_b_gpio,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_channel(runtime->unit, &channel_a_config, &runtime->channel_a), TAG, "pcnt channel a");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(runtime->channel_a,
                                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE),
                        TAG,
                        "pcnt edge a");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(runtime->channel_a,
                                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
                        TAG,
                        "pcnt level a");

    pcnt_chan_config_t channel_b_config = {
        .edge_gpio_num = config->channel_b_gpio,
        .level_gpio_num = config->channel_a_gpio,
    };
    ESP_RETURN_ON_ERROR(pcnt_new_channel(runtime->unit, &channel_b_config, &runtime->channel_b), TAG, "pcnt channel b");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(runtime->channel_b,
                                                     PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                     PCNT_CHANNEL_EDGE_ACTION_DECREASE),
                        TAG,
                        "pcnt edge b");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(runtime->channel_b,
                                                      PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                      PCNT_CHANNEL_LEVEL_ACTION_INVERSE),
                        TAG,
                        "pcnt level b");

    ESP_RETURN_ON_ERROR(configure_encoder_gpio(config->channel_a_gpio), TAG, "gpio a");
    ESP_RETURN_ON_ERROR(configure_encoder_gpio(config->channel_b_gpio), TAG, "gpio b");
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(runtime->unit), TAG, "pcnt enable");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(runtime->unit), TAG, "pcnt clear");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(runtime->unit), TAG, "pcnt start");
    return ESP_OK;
}

esp_err_t encoder_sample_now(void)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const int64_t now_us = esp_timer_get_time();
    int64_t dt_us = now_us - last_sample_us;
    if (dt_us <= 0) {
        dt_us = ENCODER_SAMPLE_PERIOD_US;
    }
    last_sample_us = now_us;

    int32_t counts[ENCODER_COUNT] = {0};
    float rpm[ENCODER_COUNT] = {0};

    portENTER_CRITICAL(&state_mux);
    memcpy(counts, state.counts, sizeof(counts));
    portEXIT_CRITICAL(&state_mux);

    for (int i = 0; i < ENCODER_COUNT; ++i) {
        int pulse_count = 0;
        esp_err_t ret = pcnt_unit_get_count(encoders[i].unit, &pulse_count);
        if (ret != ESP_OK) {
            return ret;
        }
        ret = pcnt_unit_clear_count(encoders[i].unit);
        if (ret != ESP_OK) {
            return ret;
        }

        const int delta = apply_invert(pulse_count, ENCODER_CONFIGS[i].invert);
        counts[i] += delta;
        rpm[i] = ((float)delta * 60000000.0f) /
                 (ENCODER_COUNTS_PER_OUTPUT_REV * (float)dt_us);
    }

    portENTER_CRITICAL(&state_mux);
    memcpy(state.counts, counts, sizeof(state.counts));
    memcpy(state.rpm, rpm, sizeof(state.rpm));
    portEXIT_CRITICAL(&state_mux);
    return ESP_OK;
}

esp_err_t encoder_init(void)
{
    if (initialized) {
        return ESP_OK;
    }

    for (int i = 0; i < ENCODER_COUNT; ++i) {
        ESP_RETURN_ON_ERROR(configure_encoder_unit(&ENCODER_CONFIGS[i], &encoders[i]), TAG, "encoder unit");
    }

    memset(&state, 0, sizeof(state));
    last_sample_us = esp_timer_get_time();

    initialized = true;
    ESP_LOGI(TAG,
             "Encoders iniciados L=%d/%d R=%d/%d CPR=%.1f sample=controle",
             ENCODER_LEFT_A_GPIO,
             ENCODER_LEFT_B_GPIO,
             ENCODER_RIGHT_A_GPIO,
             ENCODER_RIGHT_B_GPIO,
             ENCODER_COUNTS_PER_OUTPUT_REV);
    return ESP_OK;
}

esp_err_t encoder_reset(void)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    for (int i = 0; i < ENCODER_COUNT; ++i) {
        ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(encoders[i].unit), TAG, "pcnt clear reset");
    }

    portENTER_CRITICAL(&state_mux);
    memset(&state, 0, sizeof(state));
    portEXIT_CRITICAL(&state_mux);
    last_sample_us = esp_timer_get_time();
    ESP_LOGI(TAG, "Encoders resetados");
    return ESP_OK;
}

bool encoder_get_state(encoder_state_t *out_state)
{
    if (out_state == NULL || !initialized) {
        return false;
    }

    portENTER_CRITICAL(&state_mux);
    *out_state = state;
    portEXIT_CRITICAL(&state_mux);
    return true;
}
