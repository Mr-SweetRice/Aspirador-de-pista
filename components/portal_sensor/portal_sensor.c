#include "portal_sensor.h"

#include "control.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "portal_sensor_config.h"

#define PORTAL_TASK_PERIOD_MS 20
#define PORTAL_REARM_US 2000000LL

static const char *TAG = "portal_digital";
static TaskHandle_t portal_task_handle;
static portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;
static portal_sensor_state_t state = {
    .enabled = false,
    .sensor_ok = false,
    .threshold_mm = 300,
    .stop_speed_percent = 20,
    .stop_delay_ms = 500,
};
static bool initialized;
static bool reporting_ready;
static int64_t rearm_at_us;
static int64_t stop_at_us;

static void IRAM_ATTR portal_gpio_isr(void *arg)
{
    (void)arg;
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (portal_task_handle != NULL) {
        vTaskNotifyGiveFromISR(portal_task_handle, &higher_priority_task_woken);
    }
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t configure_portal_gpio(void)
{
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << PORTAL_SENSOR_GPIO1_INPUT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&config), TAG, "configuracao da entrada digital");

    const esp_err_t isr_service = gpio_install_isr_service(0);
    if (isr_service != ESP_OK && isr_service != ESP_ERR_INVALID_STATE) {
        return isr_service;
    }
    ESP_RETURN_ON_ERROR(
        gpio_isr_handler_add(PORTAL_SENSOR_GPIO1_INPUT, portal_gpio_isr, NULL),
        TAG,
        "interrupcao da entrada digital");
    return ESP_OK;
}

static void portal_task(void *arg)
{
    (void)arg;
    while (true) {
        const uint32_t edge_count =
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(PORTAL_TASK_PERIOD_MS));
        const int64_t now = esp_timer_get_time();
        const bool input_active =
            gpio_get_level(PORTAL_SENSOR_GPIO1_INPUT) == PORTAL_SENSOR_GPIO1_ACTIVE_LEVEL;

        control_navigation_state_t navigation = {0};
        const bool running = control_get_navigation_state(&navigation) && navigation.running;
        bool begin_stop = false;
        bool stop_now = false;
        uint8_t begin_stop_speed = 0;
        uint8_t counted = 0;
        uint32_t counted_edges = 0;

        portENTER_CRITICAL(&state_mux);
        state.sensor_ok = true;
        state.gpio1_active = input_active;
        state.gpio1_events += edge_count;
        state.detected = input_active;

        if (!running) {
            state.count = 0;
            state.stopping = false;
            stop_at_us = 0;
        } else if (edge_count > 0 && state.enabled &&
                   now >= rearm_at_us && !state.stopping) {
            ++state.count;
            rearm_at_us = now + PORTAL_REARM_US;
            counted = state.count;
            counted_edges = state.gpio1_events;
            if (state.count >= 2) {
                state.stopping = true;
                stop_at_us = now + ((int64_t)state.stop_delay_ms * 1000);
                begin_stop = true;
                begin_stop_speed = state.stop_speed_percent;
            }
        }

        stop_now = running && state.stopping && now >= stop_at_us;
        portEXIT_CRITICAL(&state_mux);

        if (counted != 0) {
            ESP_LOGI(TAG, "Portal contado por GPIO%d: %u/2 (ativo=%d, bordas=%lu)",
                     PORTAL_SENSOR_GPIO1_INPUT, (unsigned)counted, input_active, (unsigned long)counted_edges);
        }
        if (begin_stop) {
            control_begin_portal_stop(begin_stop_speed);
        }
        if (stop_now) {
            control_stop_navigation();
            portENTER_CRITICAL(&state_mux);
            state.stopping = false;
            state.count = 0;
            portEXIT_CRITICAL(&state_mux);
            ESP_LOGI(TAG, "Parada por dois pulsos digitais concluida");
        }
    }
}

esp_err_t portal_sensor_init(void)
{
    if (initialized) return ESP_OK;
    ESP_RETURN_ON_ERROR(configure_portal_gpio(), TAG, "entrada digital do portal");
    if (xTaskCreatePinnedToCore(
            portal_task,
            "portal_digital",
            3072,
            NULL,
            8,
            &portal_task_handle,
            0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    portENTER_CRITICAL(&state_mux);
    state.sensor_ok = true;
    state.gpio1_active =
        gpio_get_level(PORTAL_SENSOR_GPIO1_INPUT) == PORTAL_SENSOR_GPIO1_ACTIVE_LEVEL;
    portEXIT_CRITICAL(&state_mux);
    reporting_ready = true;
    initialized = true;
    ESP_LOGI(TAG,
             "Contagem digital iniciada: sensor GPIO1/OUT -> ESP GPIO%d, ativo em nivel baixo",
             PORTAL_SENSOR_GPIO1_INPUT);
    return ESP_OK;
}

esp_err_t portal_sensor_set_config(bool enabled,
                                   uint16_t threshold_mm,
                                   uint8_t stop_speed_percent,
                                   uint16_t stop_delay_ms)
{
    if (stop_speed_percent > 100 || stop_delay_ms > 10000) return ESP_ERR_INVALID_ARG;
    portENTER_CRITICAL(&state_mux);
    state.enabled = enabled;
    state.threshold_mm = threshold_mm;
    state.stop_speed_percent = stop_speed_percent;
    state.stop_delay_ms = stop_delay_ms;
    state.count = 0;
    state.stopping = false;
    portEXIT_CRITICAL(&state_mux);
    rearm_at_us = 0;
    stop_at_us = 0;
    return ESP_OK;
}

bool portal_sensor_get_state(portal_sensor_state_t *out_state)
{
    if (out_state == NULL) return false;
    portENTER_CRITICAL(&state_mux);
    *out_state = state;
    portEXIT_CRITICAL(&state_mux);
    return reporting_ready;
}
