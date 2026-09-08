#ifndef MOTORS_CONFIG_H
#define MOTORS_CONFIG_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "encoder_config.h"

typedef enum {
    MOTORS_MOTOR_LEFT = 0,
    MOTORS_MOTOR_RIGHT,
    MOTORS_MOTOR_AUX,
    MOTORS_MOTOR_COUNT,
} motors_motor_id_t;

typedef enum {
    MOTORS_DRIVER_TB6612 = 0,
    MOTORS_DRIVER_SI2300,
} motors_driver_t;

typedef struct {
    motors_motor_id_t id;
    motors_driver_t driver;
    gpio_num_t pwm_gpio;
    gpio_num_t in1_gpio;
    gpio_num_t in2_gpio;
    gpio_num_t encoder_a_gpio;
    gpio_num_t encoder_b_gpio;
    ledc_channel_t pwm_channel;
    int direction_invert;
    int encoder_invert;
    int has_direction;
    int has_encoder;
} motors_config_t;

#define MOTORS_LEFT_PWMA_GPIO GPIO_NUM_39
#define MOTORS_LEFT_AIN1_GPIO GPIO_NUM_40
#define MOTORS_LEFT_AIN2_GPIO GPIO_NUM_41

#define MOTORS_RIGHT_PWMB_GPIO GPIO_NUM_1
#define MOTORS_RIGHT_BIN1_GPIO GPIO_NUM_2
#define MOTORS_RIGHT_BIN2_GPIO GPIO_NUM_42

#define MOTORS_TB6612_STBY_GPIO GPIO_NUM_36
#define MOTORS_AUX_PWM_GPIO GPIO_NUM_4

#define MOTORS_PWM_MODE LEDC_LOW_SPEED_MODE
#define MOTORS_PWM_TIMER LEDC_TIMER_0
#define MOTORS_PWM_DUTY_RES LEDC_TIMER_10_BIT
#define MOTORS_PWM_FREQ_HZ 25000
#define MOTORS_PWM_MAX_DUTY ((1 << 10) - 1)

#define MOTORS_LEFT_PWM_CHANNEL LEDC_CHANNEL_0
#define MOTORS_RIGHT_PWM_CHANNEL LEDC_CHANNEL_1
#define MOTORS_AUX_PWM_CHANNEL LEDC_CHANNEL_2

#define MOTORS_DEFAULT_SPEED_PERCENT 50
#define MOTORS_BATTERY_GUARD_TASK_CORE_ID 1
#define MOTORS_BATTERY_GUARD_TASK_PRIORITY 2

static const motors_config_t MOTORS_CONFIGS[MOTORS_MOTOR_COUNT] = {
    [MOTORS_MOTOR_LEFT] = {
        .id = MOTORS_MOTOR_LEFT,
        .driver = MOTORS_DRIVER_TB6612,
        .pwm_gpio = MOTORS_LEFT_PWMA_GPIO,
        .in1_gpio = MOTORS_LEFT_AIN1_GPIO,
        .in2_gpio = MOTORS_LEFT_AIN2_GPIO,
        .encoder_a_gpio = ENCODER_LEFT_A_GPIO,
        .encoder_b_gpio = ENCODER_LEFT_B_GPIO,
        .pwm_channel = MOTORS_LEFT_PWM_CHANNEL,
        .direction_invert = 0,
        .encoder_invert = ENCODER_LEFT_INVERT,
        .has_direction = 1,
        .has_encoder = 1,
    },
    [MOTORS_MOTOR_RIGHT] = {
        .id = MOTORS_MOTOR_RIGHT,
        .driver = MOTORS_DRIVER_TB6612,
        .pwm_gpio = MOTORS_RIGHT_PWMB_GPIO,
        .in1_gpio = MOTORS_RIGHT_BIN1_GPIO,
        .in2_gpio = MOTORS_RIGHT_BIN2_GPIO,
        .encoder_a_gpio = ENCODER_RIGHT_A_GPIO,
        .encoder_b_gpio = ENCODER_RIGHT_B_GPIO,
        .pwm_channel = MOTORS_RIGHT_PWM_CHANNEL,
        .direction_invert = 0,
        .encoder_invert = ENCODER_RIGHT_INVERT,
        .has_direction = 1,
        .has_encoder = 1,
    },
    [MOTORS_MOTOR_AUX] = {
        .id = MOTORS_MOTOR_AUX,
        .driver = MOTORS_DRIVER_SI2300,
        .pwm_gpio = MOTORS_AUX_PWM_GPIO,
        .in1_gpio = GPIO_NUM_NC,
        .in2_gpio = GPIO_NUM_NC,
        .encoder_a_gpio = GPIO_NUM_NC,
        .encoder_b_gpio = GPIO_NUM_NC,
        .pwm_channel = MOTORS_AUX_PWM_CHANNEL,
        .direction_invert = 0,
        .encoder_invert = 0,
        .has_direction = 0,
        .has_encoder = 0,
    },
};

#endif
