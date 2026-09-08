#ifndef SAFETY_H
#define SAFETY_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    bool collision_enabled;
    bool battery_block_enabled;
    bool line_loss_enabled;
    bool ble_loss_enabled;
    bool distance_limit_enabled;
    bool collision_active;
    bool battery_block_active;
    bool line_loss_active;
    bool ble_loss_active;
    bool distance_limit_active;
    bool motors_blocked;
    bool line_visible;
    bool line_seen_once;
    bool ble_connected;
    float roll_limit_deg;
    float battery_block_percent;
    float line_loss_timeout_s;
    float line_loss_elapsed_s;
    float current_roll_deg;
    float current_battery_percent;
    float distance_limit_m;
    float distance_traveled_m;
} safety_state_t;

esp_err_t safety_init(void);
bool safety_motors_allowed(void);
bool safety_get_state(safety_state_t *out_state);
esp_err_t safety_set_collision_enabled(bool enabled);
esp_err_t safety_set_battery_block_enabled(bool enabled);
esp_err_t safety_set_line_loss_enabled(bool enabled);
esp_err_t safety_set_ble_loss_enabled(bool enabled);
esp_err_t safety_set_distance_limit_enabled(bool enabled);
esp_err_t safety_set_ble_connected(bool connected);
esp_err_t safety_set_roll_limit_deg(float limit_deg);
esp_err_t safety_set_battery_block_percent(float percent);
esp_err_t safety_set_line_loss_timeout_s(float timeout_s);
esp_err_t safety_set_distance_limit_m(float distance_m);
esp_err_t safety_reset_distance(void);

#endif
