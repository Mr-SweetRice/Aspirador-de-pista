#ifndef PORTAL_SENSOR_H
#define PORTAL_SENSOR_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    bool enabled;
    bool sensor_ok;
    bool detected;
    bool stopping;
    bool gpio1_active;
    uint8_t count;
    uint32_t gpio1_events;
    uint16_t distance_mm;
    uint16_t threshold_mm;
    uint8_t stop_speed_percent;
    uint16_t stop_delay_ms;
} portal_sensor_state_t;

esp_err_t portal_sensor_init(void);
esp_err_t portal_sensor_set_config(bool enabled, uint16_t threshold_mm, uint8_t stop_speed_percent, uint16_t stop_delay_ms);
bool portal_sensor_get_state(portal_sensor_state_t *out_state);

#endif
