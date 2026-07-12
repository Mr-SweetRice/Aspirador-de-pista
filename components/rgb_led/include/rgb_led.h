#ifndef RGB_LED_H
#define RGB_LED_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    RGB_LED_MODE_DISABLED = 0,
    RGB_LED_MODE_BATTERY = 1,
    RGB_LED_MODE_MANUAL = 2,
    RGB_LED_MODE_RACE_PLAN = 3,
} rgb_led_mode_t;

typedef struct {
    rgb_led_mode_t mode;
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    uint8_t intensity;
    bool enabled;
} rgb_led_state_t;

esp_err_t rgb_led_init(void);
esp_err_t rgb_led_set_enabled(bool enabled);
esp_err_t rgb_led_set_mode(rgb_led_mode_t mode);
esp_err_t rgb_led_set_manual_color(uint8_t red, uint8_t green, uint8_t blue, uint8_t intensity);
esp_err_t rgb_led_set_race_plan_color(uint8_t red, uint8_t green, uint8_t blue);
bool rgb_led_get_state(rgb_led_state_t *out_state);

#endif
