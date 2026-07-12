#ifndef ENCODER_CONFIG_H
#define ENCODER_CONFIG_H

#include "driver/gpio.h"

#define ENCODER_LEFT_A_GPIO GPIO_NUM_35
#define ENCODER_LEFT_B_GPIO GPIO_NUM_14
#define ENCODER_RIGHT_A_GPIO GPIO_NUM_21
#define ENCODER_RIGHT_B_GPIO GPIO_NUM_5

#define ENCODER_LEFT_INVERT 0
#define ENCODER_RIGHT_INVERT 0
#define ENCODER_COUNTS_PER_MOTOR_REV 11.0f
#define ENCODER_MOTOR_GEAR_REDUCTION 13.0f
#define ENCODER_QUADRATURE_MULTIPLIER 4.0f
#define ENCODER_COUNTS_PER_OUTPUT_REV \
    (ENCODER_COUNTS_PER_MOTOR_REV * ENCODER_MOTOR_GEAR_REDUCTION * ENCODER_QUADRATURE_MULTIPLIER)

#define ENCODER_PCNT_GLITCH_FILTER_NS 1000
#define ENCODER_SAMPLE_TARGET_HZ 1000
#define ENCODER_SAMPLE_PERIOD_US (1000000 / ENCODER_SAMPLE_TARGET_HZ)
#define ENCODER_RPM_WINDOW_MS 100

typedef enum {
    ENCODER_LEFT = 0,
    ENCODER_RIGHT,
    ENCODER_COUNT,
} encoder_id_t;

typedef struct {
    encoder_id_t id;
    gpio_num_t channel_a_gpio;
    gpio_num_t channel_b_gpio;
    int invert;
} encoder_channel_config_t;

static const encoder_channel_config_t ENCODER_CONFIGS[ENCODER_COUNT] = {
    [ENCODER_LEFT] = {
        .id = ENCODER_LEFT,
        .channel_a_gpio = ENCODER_LEFT_A_GPIO,
        .channel_b_gpio = ENCODER_LEFT_B_GPIO,
        .invert = ENCODER_LEFT_INVERT,
    },
    [ENCODER_RIGHT] = {
        .id = ENCODER_RIGHT,
        .channel_a_gpio = ENCODER_RIGHT_A_GPIO,
        .channel_b_gpio = ENCODER_RIGHT_B_GPIO,
        .invert = ENCODER_RIGHT_INVERT,
    },
};

#endif
