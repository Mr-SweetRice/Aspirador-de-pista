#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "line_sensor_config.h"

typedef enum {
    LINE_SENSOR_TRACK_BLACK = 0,
    LINE_SENSOR_TRACK_WHITE = 1,
} line_sensor_track_type_t;

typedef struct {
    uint16_t raw[LINE_SENSOR_QTR_COUNT];
    uint16_t calibrated[LINE_SENSOR_QTR_COUNT];
    uint16_t line_values[LINE_SENSOR_QTR_COUNT];
    uint16_t position;
    bool line_visible;
    bool calibrated_valid;
    bool calibrating;
    line_sensor_track_type_t track_type;
    uint8_t threshold_percent;
    float read_hz;
} line_sensor_state_t;

esp_err_t line_sensor_init(void);
esp_err_t line_sensor_sample_now(void);
esp_err_t line_sensor_start_calibration(uint32_t duration_ms);
esp_err_t line_sensor_set_track_type(line_sensor_track_type_t track_type);
esp_err_t line_sensor_set_threshold_percent(uint8_t threshold_percent);
line_sensor_track_type_t line_sensor_get_track_type(void);
bool line_sensor_get_state(line_sensor_state_t *out_state);

#endif
