#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/i2c_master.h"

typedef struct {
    float accel_mps2[3];
    float gyro_dps[3];
    float mag_ut[3];
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float mag_yaw_deg;
    float mag_yaw_xy_deg;
    float mag_yaw_xz_deg;
    float mag_yaw_yz_deg;
    float mag_yaw_error_deg;
    float mag_field_norm_ut;
    float mag_filter_gain;
    float yaw_drift_threshold_dps;
    float sample_hz;
    uint8_t mag_heading_mode;
    float quat_wxyz[4];
    bool mag_ignored;
    bool accel_gyro_calibrating;
    bool accel_gyro_calibrated;
    bool yaw_drift_calibrating;
    bool yaw_drift_calibrated;
    bool mag_calibrating;
    bool mag_calibrated;
} imu_state_t;

esp_err_t imu_init(void);
esp_err_t imu_start_accel_gyro_calibration(uint32_t duration_ms);
esp_err_t imu_start_yaw_drift_calibration(uint32_t duration_ms);
esp_err_t imu_start_mag_calibration(uint32_t duration_ms);
esp_err_t imu_start_full_calibration(uint32_t duration_ms);
esp_err_t imu_set_mag_filter_gain(float gain);
float imu_get_mag_filter_gain(void);
esp_err_t imu_set_mag_heading_mode(uint8_t mode);
uint8_t imu_get_mag_heading_mode(void);
esp_err_t imu_set_mag_ignored(bool ignored);
bool imu_get_mag_ignored(void);
esp_err_t imu_set_yaw_drift_threshold(float threshold_dps);
float imu_get_yaw_drift_threshold(void);
esp_err_t imu_reset_yaw(void);
bool imu_get_state(imu_state_t *out_state);
i2c_master_bus_handle_t imu_get_i2c_bus(void);
bool imu_i2c_lock(uint32_t timeout_ms);
void imu_i2c_unlock(void);

#endif
