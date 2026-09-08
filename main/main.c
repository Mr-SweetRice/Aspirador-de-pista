#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "comms_ble.h"
#include "battery_level.h"
#include "control.h"
#include "encoder.h"
#include "imu.h"
#include "line_sensor.h"
#include "memory_maps.h"
#include "motors.h"
#include "odometry.h"
#include "portal_sensor.h"
#include "rgb_led.h"
#include "safety.h"

#define ROBOT_TASK_CORE_LINE_SENSOR 0
#define ROBOT_TASK_CORE_CONTROL 1
#define ROBOT_TASK_CORE_COMMS 1

static const char *TAG = "main";

static bool init_or_warn(const char *name, esp_err_t ret)
{
    if (ret == ESP_OK) {
        return true;
    }

    ESP_LOGW(TAG, "%s nao iniciou (%s). BLE continua ativo para diagnostico.", name, esp_err_to_name(ret));
    return false;
}

static void storage_init(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);
}

void app_main(void)
{
    storage_init();
    ESP_ERROR_CHECK(memory_maps_init());

    init_or_warn("encoder", encoder_init());
    init_or_warn("bateria", battery_level_init());
    init_or_warn("led rgb", rgb_led_init());
    init_or_warn("sensor de linha", line_sensor_init());
    init_or_warn("IMU", imu_init());
    init_or_warn("entrada digital de portal", portal_sensor_init());
    init_or_warn("odometria", odometry_init());
    init_or_warn("seguranca", safety_init());
    init_or_warn("controle", control_init());
    ESP_ERROR_CHECK(comms_ble_init());

    ESP_LOGI(TAG,
             "Robo corretamente iniciado. Nucleos: linha=%d control=%d comms=%d",
             ROBOT_TASK_CORE_LINE_SENSOR,
             ROBOT_TASK_CORE_CONTROL,
             ROBOT_TASK_CORE_COMMS);
}
