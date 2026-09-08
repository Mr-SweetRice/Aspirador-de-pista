#ifndef MOTORS_H
#define MOTORS_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "motors_config.h"

esp_err_t motors_init(void);
esp_err_t motors_set_percent(motors_motor_id_t motor, int percent);
esp_err_t motors_stop_all(void);
esp_err_t motors_stop_all_immediate(void);
esp_err_t motors_brake_drive(void);
esp_err_t motors_brake_drive_for_ms(uint32_t duration_ms);
esp_err_t motors_set_zero_brake_enabled(bool enabled);
bool motors_get_zero_brake_enabled(void);

#endif
