#ifndef PORTAL_SENSOR_CONFIG_H
#define PORTAL_SENSOR_CONFIG_H

#include "driver/gpio.h"

/* Conectar o pino GPIO1/INT do modulo VL53L0X neste pino do ESP32-S3. */
#define PORTAL_SENSOR_GPIO1_INPUT GPIO_NUM_12
#define PORTAL_SENSOR_GPIO1_ACTIVE_LEVEL 0

#endif
