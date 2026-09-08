#include "motors.h"

#include <stdbool.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "memory_config.h"
#include "nvs.h"
#include "safety.h"

static const char *TAG = "motors";
static const char *ZERO_BRAKE_NVS_KEY = "mot_zero_brk";

static bool initialized;
static bool zero_brake_enabled = true;
static int motor_percent[MOTORS_MOTOR_COUNT];
static TaskHandle_t safety_guard_task_handle;
static esp_timer_handle_t brake_release_timer_handle;

static uint32_t percent_to_duty(int percent)
{
    int magnitude = abs(percent);
    if (magnitude > 100) {
        magnitude = 100;
    }

    return ((uint32_t)magnitude * MOTORS_PWM_MAX_DUTY) / 100U;
}

static esp_err_t configure_output_gpio(gpio_num_t gpio)
{
    if (gpio == GPIO_NUM_NC) {
        return ESP_OK;
    }

    gpio_config_t config = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&config), TAG, "gpio config");
    ESP_RETURN_ON_ERROR(gpio_set_level(gpio, 0), TAG, "gpio low");
    return ESP_OK;
}

static esp_err_t set_direction(const motors_config_t *config, int percent)
{
    if (!config->has_direction) {
        return ESP_OK;
    }

    int direction = percent;
    if (config->direction_invert) {
        direction = -direction;
    }

    int in1 = 0;
    int in2 = 0;
    if (direction > 0) {
        in1 = 1;
    } else if (direction < 0) {
        in2 = 1;
    }

    ESP_RETURN_ON_ERROR(gpio_set_level(config->in1_gpio, in1), TAG, "in1");
    ESP_RETURN_ON_ERROR(gpio_set_level(config->in2_gpio, in2), TAG, "in2");
    return ESP_OK;
}

static esp_err_t set_tb6612_short_brake(const motors_config_t *config)
{
    if (!config->has_direction || config->driver != MOTORS_DRIVER_TB6612) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(gpio_set_level(MOTORS_TB6612_STBY_GPIO, 1), TAG, "stby high");
    ESP_RETURN_ON_ERROR(gpio_set_level(config->in1_gpio, 1), TAG, "brake in1");
    ESP_RETURN_ON_ERROR(gpio_set_level(config->in2_gpio, 1), TAG, "brake in2");
    ESP_RETURN_ON_ERROR(ledc_set_duty(MOTORS_PWM_MODE, config->pwm_channel, MOTORS_PWM_MAX_DUTY), TAG, "brake duty");
    ESP_RETURN_ON_ERROR(ledc_update_duty(MOTORS_PWM_MODE, config->pwm_channel), TAG, "brake update duty");
    return ESP_OK;
}

static esp_err_t set_tb6612_coast(const motors_config_t *config)
{
    if (!config->has_direction || config->driver != MOTORS_DRIVER_TB6612) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(gpio_set_level(MOTORS_TB6612_STBY_GPIO, 1), TAG, "stby high");
    ESP_RETURN_ON_ERROR(gpio_set_level(config->in1_gpio, 0), TAG, "coast in1");
    ESP_RETURN_ON_ERROR(gpio_set_level(config->in2_gpio, 0), TAG, "coast in2");
    ESP_RETURN_ON_ERROR(ledc_set_duty(MOTORS_PWM_MODE, config->pwm_channel, MOTORS_PWM_MAX_DUTY), TAG, "coast duty");
    ESP_RETURN_ON_ERROR(ledc_update_duty(MOTORS_PWM_MODE, config->pwm_channel), TAG, "coast update duty");
    return ESP_OK;
}

static void load_zero_brake_from_nvs(void)
{
    nvs_handle_t handle;
    uint8_t stored = zero_brake_enabled ? 1U : 0U;
    esp_err_t ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        return;
    }
    if (nvs_get_u8(handle, ZERO_BRAKE_NVS_KEY, &stored) == ESP_OK) {
        zero_brake_enabled = stored != 0;
    }
    nvs_close(handle);
}

static esp_err_t save_zero_brake_to_nvs(bool enabled)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = nvs_set_u8(handle, ZERO_BRAKE_NVS_KEY, enabled ? 1U : 0U);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    return ret;
}

static void brake_release_timer_cb(void *arg)
{
    (void)arg;

    esp_err_t ret = ESP_OK;
    for (int i = 0; i < MOTORS_MOTOR_COUNT; ++i) {
        const motors_config_t *config = &MOTORS_CONFIGS[i];
        if (config->driver != MOTORS_DRIVER_TB6612 || !config->has_direction) {
            continue;
        }
        esp_err_t motor_ret = set_tb6612_coast(config);
        motor_percent[i] = 0;
        if (ret == ESP_OK) {
            ret = motor_ret;
        }
    }
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Falha ao liberar freio TB6612: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "Freio TB6612 liberado por timeout");
}

static void cancel_brake_release_timer(void)
{
    if (brake_release_timer_handle != NULL) {
        esp_timer_stop(brake_release_timer_handle);
    }
}

static void safety_guard_task(void *arg)
{
    (void)arg;

    while (true) {
        bool any_running = false;
        for (int i = 0; i < MOTORS_MOTOR_COUNT; ++i) {
            if (motor_percent[i] != 0) {
                any_running = true;
                break;
            }
        }
        if (any_running && !safety_motors_allowed()) {
            ESP_LOGW(TAG, "Parando motores: bloqueio de seguranca ativo");
            safety_state_t safety = {0};
            if (safety_get_state(&safety) && safety.distance_limit_active) {
                motors_brake_drive();
            } else {
                motors_stop_all_immediate();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t motors_init(void)
{
    if (initialized) {
        return ESP_OK;
    }

    load_zero_brake_from_nvs();

    ledc_timer_config_t timer = {
        .speed_mode = MOTORS_PWM_MODE,
        .duty_resolution = MOTORS_PWM_DUTY_RES,
        .timer_num = MOTORS_PWM_TIMER,
        .freq_hz = MOTORS_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "ledc timer");

    ESP_RETURN_ON_ERROR(configure_output_gpio(MOTORS_TB6612_STBY_GPIO), TAG, "stby");
    ESP_RETURN_ON_ERROR(gpio_set_level(MOTORS_TB6612_STBY_GPIO, 1), TAG, "stby high");

    const esp_timer_create_args_t brake_timer_args = {
        .callback = brake_release_timer_cb,
        .name = "tb6612_brake_release",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&brake_timer_args, &brake_release_timer_handle), TAG, "brake release timer");

    for (int i = 0; i < MOTORS_MOTOR_COUNT; ++i) {
        const motors_config_t *config = &MOTORS_CONFIGS[i];

        ESP_RETURN_ON_ERROR(configure_output_gpio(config->in1_gpio), TAG, "in1 config");
        ESP_RETURN_ON_ERROR(configure_output_gpio(config->in2_gpio), TAG, "in2 config");

        ledc_channel_config_t channel = {
            .gpio_num = config->pwm_gpio,
            .speed_mode = MOTORS_PWM_MODE,
            .channel = config->pwm_channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = MOTORS_PWM_TIMER,
            .duty = (config->driver == MOTORS_DRIVER_TB6612 && config->has_direction) ? MOTORS_PWM_MAX_DUTY : 0,
            .hpoint = 0,
        };
        ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), TAG, "ledc channel");
        motor_percent[i] = 0;
    }

    BaseType_t created = xTaskCreatePinnedToCore(safety_guard_task,
                                                 "motor_safe_guard",
                                                 2048,
                                                 NULL,
                                                 MOTORS_BATTERY_GUARD_TASK_PRIORITY,
                                                 &safety_guard_task_handle,
                                                 MOTORS_BATTERY_GUARD_TASK_CORE_ID);
    if (created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    initialized = true;
    ESP_LOGI(TAG,
             "Motores iniciados PWM=%u Hz L=%d R=%d AUX=%d STBY=%d guard_core=%d zero=%s",
             MOTORS_PWM_FREQ_HZ,
             MOTORS_LEFT_PWMA_GPIO,
             MOTORS_RIGHT_PWMB_GPIO,
             MOTORS_AUX_PWM_GPIO,
             MOTORS_TB6612_STBY_GPIO,
             MOTORS_BATTERY_GUARD_TASK_CORE_ID,
             zero_brake_enabled ? "brake" : "coast");
    return ESP_OK;
}

esp_err_t motors_set_percent(motors_motor_id_t motor, int percent)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (motor < 0 || motor >= MOTORS_MOTOR_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    const motors_config_t *config = &MOTORS_CONFIGS[motor];
    if (!config->has_direction && percent < 0) {
        percent = 0;
    }
    if (percent > 100) {
        percent = 100;
    } else if (percent < -100) {
        percent = -100;
    }
    if (percent != 0 && !safety_motors_allowed()) {
        ESP_LOGW(TAG, "Motor %d bloqueado: seguranca ativa", (int)motor);
        return ESP_ERR_INVALID_STATE;
    }
    if (percent != 0 && motor_percent[motor] == percent) {
        return ESP_OK;
    }
    if (percent != 0 && config->driver == MOTORS_DRIVER_TB6612 && config->has_direction) {
        cancel_brake_release_timer();
    }

    if (percent == 0 && config->driver == MOTORS_DRIVER_TB6612 && config->has_direction) {
        ESP_RETURN_ON_ERROR(zero_brake_enabled ? set_tb6612_short_brake(config) : set_tb6612_coast(config),
                            TAG,
                            "zero output");
        motor_percent[motor] = 0;
        ESP_LOGD(TAG, "Motor %d zero=%s", (int)motor, zero_brake_enabled ? "brake" : "coast");
        return ESP_OK;
    }

    const uint32_t duty = percent_to_duty(percent);
    ESP_RETURN_ON_ERROR(set_direction(config, percent), TAG, "direction");
    ESP_RETURN_ON_ERROR(ledc_set_duty(MOTORS_PWM_MODE, config->pwm_channel, duty), TAG, "duty");
    ESP_RETURN_ON_ERROR(ledc_update_duty(MOTORS_PWM_MODE, config->pwm_channel), TAG, "update duty");

    motor_percent[motor] = percent;
    ESP_LOGD(TAG, "Motor %d pwm=%d%% duty=%lu/%u", (int)motor, percent, (unsigned long)duty, MOTORS_PWM_MAX_DUTY);
    return ESP_OK;
}

esp_err_t motors_stop_all(void)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = ESP_OK;
    for (int i = 0; i < MOTORS_MOTOR_COUNT; ++i) {
        esp_err_t motor_ret = motors_set_percent((motors_motor_id_t)i, 0);
        if (ret == ESP_OK) {
            ret = motor_ret;
        }
    }

    return ret;
}

esp_err_t motors_stop_all_immediate(void)
{
    return motors_stop_all();
}

esp_err_t motors_brake_drive(void)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    cancel_brake_release_timer();

    esp_err_t ret = ESP_OK;
    for (int i = 0; i < MOTORS_MOTOR_COUNT; ++i) {
        const motors_config_t *config = &MOTORS_CONFIGS[i];
        if (config->driver != MOTORS_DRIVER_TB6612 || !config->has_direction) {
            continue;
        }
        esp_err_t motor_ret = set_tb6612_short_brake(config);
        motor_percent[i] = 0;
        if (ret == ESP_OK) {
            ret = motor_ret;
        }
    }

    return ret;
}

esp_err_t motors_brake_drive_for_ms(uint32_t duration_ms)
{
    esp_err_t ret = motors_brake_drive();
    if (ret != ESP_OK) {
        return ret;
    }

    cancel_brake_release_timer();
    if (duration_ms > 0 && brake_release_timer_handle != NULL) {
        ret = esp_timer_start_once(brake_release_timer_handle, (uint64_t)duration_ms * 1000ULL);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Falha ao agendar liberacao do freio: %s", esp_err_to_name(ret));
        }
    }
    return ret;
}

esp_err_t motors_set_zero_brake_enabled(bool enabled)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (zero_brake_enabled == enabled) {
        return ESP_OK;
    }

    esp_err_t ret = save_zero_brake_to_nvs(enabled);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Falha ao salvar freio em comando 0%%: %s", esp_err_to_name(ret));
        return ret;
    }

    zero_brake_enabled = enabled;
    ESP_LOGI(TAG, "Freio em comando 0%% %s", enabled ? "habilitado" : "desabilitado");
    return ESP_OK;
}

bool motors_get_zero_brake_enabled(void)
{
    return zero_brake_enabled;
}
