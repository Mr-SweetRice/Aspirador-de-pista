#ifndef ENCODER_H
#define ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#include "encoder_config.h"
#include "esp_err.h"

typedef struct {
    int32_t counts[ENCODER_COUNT];
    float rpm[ENCODER_COUNT];
} encoder_state_t;

esp_err_t encoder_init(void);
esp_err_t encoder_reset(void);
esp_err_t encoder_sample_now(void);
bool encoder_get_state(encoder_state_t *out_state);

#endif
