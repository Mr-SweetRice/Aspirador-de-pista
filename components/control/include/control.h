#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "motors.h"

typedef motors_motor_id_t control_motor_id_t;

typedef enum {
    CONTROL_NAV_MODE_ODOMETRY = 0,
    CONTROL_NAV_MODE_LINE = 1,
    CONTROL_NAV_MODE_AUTO_TRACK = 2,
} control_navigation_mode_t;

typedef enum {
    CONTROL_RACE_SEGMENT_NORMAL = 0,
    CONTROL_RACE_SEGMENT_CURVE = 1,
    CONTROL_RACE_SEGMENT_STOP = 2,
    CONTROL_RACE_SEGMENT_INTERSECTION = 3,
} control_race_segment_type_t;

typedef struct __attribute__((packed)) {
    uint8_t speed_percent;
    float kp;
    float ki;
    float kd;
    uint8_t aux_percent;
} control_speed_profile_point_t;

typedef struct __attribute__((packed)) {
    uint16_t start_index;
    uint16_t end_index;
    uint8_t type;
    uint8_t speed_percent;
    uint8_t max_speed_percent;
    uint8_t aux_percent;
    float kp;
    float ki;
    float kd;
} control_race_plan_segment_t;

typedef struct __attribute__((packed)) {
    uint8_t line_loss_odometry_enabled;
} control_auto_track_config_t;

typedef struct {
    bool running;
    control_navigation_mode_t mode;
    uint8_t map_slot;
    uint16_t target_index;
    uint16_t point_count;
    int8_t speed_percent;
    uint8_t active_speed_percent;
    uint8_t aux_percent;
    uint8_t active_aux_percent;
    float target_x_m;
    float target_y_m;
    float map_x_m;
    float map_y_m;
    float map_heading_rad;
    float distance_m;
    float angle_error_rad;
    float steer_percent;
    float kp;
    float ki;
    float kd;
    float line_error_raw;
    float line_error_normalized;
    float line_proportional_term;
    float line_nonlinear_term;
    float line_derivative_raw;
    float line_derivative_filtered;
    float line_correction;
    float line_left_command;
    float line_right_command;
    float line_dt_s;
    float line_max_correction;
    float line_derivative_filter_alpha;
    uint8_t motor_limit_percent;
    float loop_hz;
    float race_plan_loop_hz;
    float track_odometry_loop_hz;
    float line_sensor_loop_hz;
    float imu_loop_hz;
    float average_speed_mps;
    float max_speed_mps;
    float race_plan_average_speed_mps;
    bool speed_profile_enabled;
    bool battery_compensation_enabled;
    bool race_segment_active;
    uint8_t race_segment_type;
    uint16_t race_segment_start_index;
    uint16_t race_segment_end_index;
    uint8_t race_segment_speed_percent;
    uint8_t race_segment_max_speed_percent;
    uint8_t race_segment_aux_percent;
} control_navigation_state_t;

#define CONTROL_MOTOR_LEFT MOTORS_MOTOR_LEFT
#define CONTROL_MOTOR_RIGHT MOTORS_MOTOR_RIGHT
#define CONTROL_MOTOR_AUX MOTORS_MOTOR_AUX
#define CONTROL_MOTOR_COUNT MOTORS_MOTOR_COUNT

esp_err_t control_init(void);
esp_err_t control_set_motor_percent(control_motor_id_t motor, int percent);
esp_err_t control_stop_all(void);
esp_err_t control_emergency_stop(void);
esp_err_t control_start_map(uint8_t map_slot, int8_t speed_percent);
esp_err_t control_start_line(int8_t speed_percent);
esp_err_t control_start_auto_track(uint8_t map_slot, int8_t speed_percent);
esp_err_t control_stop_navigation(void);
esp_err_t control_set_pid(float kp, float ki, float kd);
esp_err_t control_set_line_controller(float kp,
                                      float kn,
                                      float kd,
                                      float max_correction_percent,
                                      float base_speed_percent,
                                      float derivative_filter_alpha);
esp_err_t control_save_pid_settings(float kp, float ki, float kd, uint8_t limit_percent, uint8_t aux_percent);
esp_err_t control_set_speed_profile(const control_speed_profile_point_t *points, size_t count);
esp_err_t control_set_speed_profile_enabled(bool enabled);
esp_err_t control_set_battery_compensation_enabled(bool enabled);
esp_err_t control_set_auto_track_config(const control_auto_track_config_t *config);
esp_err_t control_set_race_plan(const control_race_plan_segment_t *segments, size_t count, bool enabled);
esp_err_t control_set_motor_limit(uint8_t limit_percent);
esp_err_t control_set_aux_percent(uint8_t aux_percent);
bool control_get_navigation_state(control_navigation_state_t *out_state);

#endif
