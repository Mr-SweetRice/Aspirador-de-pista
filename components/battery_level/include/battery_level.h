#ifndef BATTERY_LEVEL_H
#define BATTERY_LEVEL_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    uint16_t raw;
    float percent;
    float voltage_v;
    bool valid;
} battery_level_state_t;

esp_err_t battery_level_init(void);
bool battery_level_get_state(battery_level_state_t *out_state);
bool battery_level_is_critical(void);

#endif
