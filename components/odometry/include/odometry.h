#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    float x_m;
    float y_m;
    float heading_rad;
    float linear_mps;
    float angular_rad_s;
    float encoder_x_m;
    float encoder_y_m;
    float encoder_heading_rad;
    float imu_x_m;
    float imu_y_m;
    float imu_heading_rad;
    float fused_x_m;
    float fused_y_m;
    float fused_heading_rad;
    int32_t left_count;
    int32_t right_count;
    bool imu_available;
} odometry_state_t;

typedef enum {
    ODOMETRY_POSE_SOURCE_FUSED = 2,
} odometry_pose_source_t;

esp_err_t odometry_init(void);
esp_err_t odometry_update_from_sensors(void);
esp_err_t odometry_reset(void);
esp_err_t odometry_reset_heading(void);
esp_err_t odometry_set_position(float x_m, float y_m);
esp_err_t odometry_set_pose_source(odometry_pose_source_t source);
odometry_pose_source_t odometry_get_pose_source(void);
bool odometry_get_state(odometry_state_t *out_state);

#endif
