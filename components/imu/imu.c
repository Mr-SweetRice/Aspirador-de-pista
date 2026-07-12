#include "imu.h"

#include <math.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_config.h"
#include "memory_config.h"
#include "nvs.h"

#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_CONFIG 0x1A
#define MPU9250_GYRO_CONFIG 0x1B
#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_ACCEL_CONFIG2 0x1D
#define MPU9250_INT_PIN_CFG 0x37
#define MPU9250_ACCEL_XOUT_H 0x3B

#define AK8963_ST1 0x02
#define AK8963_HXL 0x03
#define AK8963_CNTL1 0x0A
#define AK8963_ASAX 0x10

#define DEG_PER_RAD 57.2957795f
#define RAD_PER_DEG 0.0174532925f
#define IMU_NVS_KEY_YAW_THRESHOLD "yaw_thr"
#define IMU_NVS_KEY_YAW_THRESHOLD_MDPS "yaw_thr_mdps"
#define IMU_NVS_KEY_YAW_GYRO_BIAS_MDPS "yaw_gz_mdps"
#define IMU_NVS_KEY_MAG_IGNORED "mag_ign"

static const char *TAG = "imu";

static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t mpu_dev;
static i2c_master_dev_handle_t mag_dev;
static TaskHandle_t imu_task_handle;
static esp_timer_handle_t imu_timer_handle;
static portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;

static imu_state_t state;
static float accel_bias[3] = {0.0f, 0.0f, 0.0f};
static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static float mag_adjust[3] = {1.0f, 1.0f, 1.0f};
static float mag_bias[3] = {0.0f, 0.0f, 0.0f};
static float mag_scale[3] = {1.0f, 1.0f, 1.0f};
static float mag_yaw_correction_gain = IMU_MAG_YAW_CORRECTION_GAIN;
static float yaw_drift_threshold_dps = IMU_YAW_DRIFT_THRESHOLD_DPS;
static uint8_t mag_heading_mode = IMU_MAG_HEADING_MODE_DEFAULT;
static bool mag_ignored = IMU_MAG_IGNORED_DEFAULT != 0;
static bool yaw_reset_requested;
static float mag_min[3];
static float mag_max[3];
static uint32_t mag_cal_samples;
static int64_t mag_cal_end_us;
static bool accel_gyro_calibrating;
static bool yaw_drift_calibrating;
static bool start_mag_after_accel_gyro;
static int64_t stationary_cal_end_us;
static int64_t yaw_drift_cal_end_us;
static uint32_t pending_mag_duration_ms;
static float accel_sum[3];
static float gyro_sum[3];
static uint32_t stationary_samples;
static float yaw_drift_sum;
static uint32_t yaw_drift_samples;
static bool initialized;

static void load_settings_from_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) {
        if (ret != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "NVS IMU indisponivel para leitura: %s", esp_err_to_name(ret));
        }
        return;
    }

    float stored_threshold = yaw_drift_threshold_dps;
    int32_t stored_threshold_mdps = 0;
    ret = nvs_get_i32(handle, IMU_NVS_KEY_YAW_THRESHOLD_MDPS, &stored_threshold_mdps);
    if (ret == ESP_OK &&
        stored_threshold_mdps >= (int32_t)(IMU_YAW_DRIFT_THRESHOLD_MIN_DPS * 1000.0f) &&
        stored_threshold_mdps <= (int32_t)(IMU_YAW_DRIFT_THRESHOLD_MAX_DPS * 1000.0f)) {
        yaw_drift_threshold_dps = (float)stored_threshold_mdps / 1000.0f;
    } else if (ret == ESP_ERR_NVS_NOT_FOUND) {
        size_t threshold_len = sizeof(stored_threshold);
        ret = nvs_get_blob(handle, IMU_NVS_KEY_YAW_THRESHOLD, &stored_threshold, &threshold_len);
        if (ret == ESP_OK &&
            threshold_len == sizeof(stored_threshold) &&
            stored_threshold >= IMU_YAW_DRIFT_THRESHOLD_MIN_DPS &&
            stored_threshold <= IMU_YAW_DRIFT_THRESHOLD_MAX_DPS) {
            yaw_drift_threshold_dps = stored_threshold;
        } else if (ret != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "Threshold yaw salvo invalido: %s", esp_err_to_name(ret));
        }
    } else if (ret != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Threshold yaw salvo invalido: %s", esp_err_to_name(ret));
    }

    uint8_t stored_mag_ignored = mag_ignored ? 1 : 0;
    ret = nvs_get_u8(handle, IMU_NVS_KEY_MAG_IGNORED, &stored_mag_ignored);
    if (ret == ESP_OK) {
        mag_ignored = stored_mag_ignored != 0;
    } else if (ret != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Estado ignorar mag salvo invalido: %s", esp_err_to_name(ret));
    }

    int32_t stored_yaw_bias_mdps = 0;
    ret = nvs_get_i32(handle, IMU_NVS_KEY_YAW_GYRO_BIAS_MDPS, &stored_yaw_bias_mdps);
    if (ret == ESP_OK) {
        gyro_bias[2] = (float)stored_yaw_bias_mdps / 1000.0f;
        state.yaw_drift_calibrated = true;
    } else if (ret != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Bias yaw salvo invalido: %s", esp_err_to_name(ret));
    }

    nvs_close(handle);
    ESP_LOGI(TAG,
             "Configuracao IMU carregada: yaw_threshold=%.3f dps yaw_gyro_bias=%.4f dps mag_ignored=%s",
             yaw_drift_threshold_dps,
             gyro_bias[2],
             mag_ignored ? "sim" : "nao");
}

static esp_err_t save_i32_to_nvs(const char *key, int32_t value)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = nvs_set_i32(handle, key, value);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    return ret;
}

static esp_err_t save_u8_to_nvs(const char *key, uint8_t value)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(MEMORY_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = nvs_set_u8(handle, key, value);
    if (ret == ESP_OK) {
        ret = nvs_commit(handle);
    }
    nvs_close(handle);
    return ret;
}

static esp_err_t write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t value)
{
    const uint8_t data[] = {reg, value};
    return i2c_master_transmit(dev, data, sizeof(data), pdMS_TO_TICKS(100));
}

static esp_err_t read_regs(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

static int16_t be16(const uint8_t *data)
{
    return (int16_t)((data[0] << 8) | data[1]);
}

static int16_t le16(const uint8_t *data)
{
    return (int16_t)((data[1] << 8) | data[0]);
}

static float wrap_180(float value)
{
    while (value > 180.0f) {
        value -= 360.0f;
    }
    while (value < -180.0f) {
        value += 360.0f;
    }
    return value;
}

static float mag_heading_from_plane(float axis_a, float axis_b)
{
    return wrap_180((atan2f(-axis_b, axis_a) * DEG_PER_RAD) + IMU_MAG_DECLINATION_DEG);
}

static void update_quaternion_from_euler(imu_state_t *next)
{
    const float half_yaw = next->yaw_deg * RAD_PER_DEG * 0.5f;
    const float half_pitch = next->pitch_deg * RAD_PER_DEG * 0.5f;
    const float half_roll = next->roll_deg * RAD_PER_DEG * 0.5f;

    const float cy = cosf(half_yaw);
    const float sy = sinf(half_yaw);
    const float cp = cosf(half_pitch);
    const float sp = sinf(half_pitch);
    const float cr = cosf(half_roll);
    const float sr = sinf(half_roll);

    next->quat_wxyz[0] = (cr * cp * cy) + (sr * sp * sy);
    next->quat_wxyz[1] = (sr * cp * cy) - (cr * sp * sy);
    next->quat_wxyz[2] = (cr * sp * cy) + (sr * cp * sy);
    next->quat_wxyz[3] = (cr * cp * sy) - (sr * sp * cy);

    const float norm = sqrtf((next->quat_wxyz[0] * next->quat_wxyz[0]) +
                             (next->quat_wxyz[1] * next->quat_wxyz[1]) +
                             (next->quat_wxyz[2] * next->quat_wxyz[2]) +
                             (next->quat_wxyz[3] * next->quat_wxyz[3]));
    if (norm > 0.0001f) {
        for (int i = 0; i < 4; ++i) {
            next->quat_wxyz[i] /= norm;
        }
    } else {
        next->quat_wxyz[0] = 1.0f;
        next->quat_wxyz[1] = 0.0f;
        next->quat_wxyz[2] = 0.0f;
        next->quat_wxyz[3] = 0.0f;
    }
}

static void sensor_to_robot_frame(const float sensor[3], float robot[3])
{
    robot[0] = (IMU_SENSOR_X_TO_ROBOT_X * sensor[0]) +
               (IMU_SENSOR_Y_TO_ROBOT_X * sensor[1]) +
               (IMU_SENSOR_Z_TO_ROBOT_X * sensor[2]);
    robot[1] = (IMU_SENSOR_X_TO_ROBOT_Y * sensor[0]) +
               (IMU_SENSOR_Y_TO_ROBOT_Y * sensor[1]) +
               (IMU_SENSOR_Z_TO_ROBOT_Y * sensor[2]);
    robot[2] = (IMU_SENSOR_X_TO_ROBOT_Z * sensor[0]) +
               (IMU_SENSOR_Y_TO_ROBOT_Z * sensor[1]) +
               (IMU_SENSOR_Z_TO_ROBOT_Z * sensor[2]);
}

static void mag_sensor_to_mpu_frame(const float sensor_mag[3], float mpu_mag[3])
{
    mpu_mag[0] = (IMU_MAG_SENSOR_X_TO_MPU_X * sensor_mag[0]) +
                 (IMU_MAG_SENSOR_Y_TO_MPU_X * sensor_mag[1]) +
                 (IMU_MAG_SENSOR_Z_TO_MPU_X * sensor_mag[2]);
    mpu_mag[1] = (IMU_MAG_SENSOR_X_TO_MPU_Y * sensor_mag[0]) +
                 (IMU_MAG_SENSOR_Y_TO_MPU_Y * sensor_mag[1]) +
                 (IMU_MAG_SENSOR_Z_TO_MPU_Y * sensor_mag[2]);
    mpu_mag[2] = (IMU_MAG_SENSOR_X_TO_MPU_Z * sensor_mag[0]) +
                 (IMU_MAG_SENSOR_Y_TO_MPU_Z * sensor_mag[1]) +
                 (IMU_MAG_SENSOR_Z_TO_MPU_Z * sensor_mag[2]);
}

static esp_err_t imu_bus_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = IMU_I2C_PORT,
        .sda_io_num = IMU_MPU9250_SDA_GPIO,
        .scl_io_num = IMU_MPU9250_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &i2c_bus), TAG, "i2c bus");

    i2c_device_config_t mpu_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_MPU9250_ADDR,
        .scl_speed_hz = IMU_I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &mpu_config, &mpu_dev), TAG, "mpu dev");

    i2c_device_config_t mag_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_AK8963_ADDR,
        .scl_speed_hz = IMU_I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &mag_config, &mag_dev), TAG, "mag dev");

    return ESP_OK;
}

static esp_err_t mpu9250_init(void)
{
    ESP_RETURN_ON_ERROR(write_reg(mpu_dev, MPU9250_PWR_MGMT_1, 0x80), TAG, "mpu reset");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(write_reg(mpu_dev, MPU9250_PWR_MGMT_1, 0x01), TAG, "mpu clock");
    ESP_RETURN_ON_ERROR(write_reg(mpu_dev, MPU9250_CONFIG, 0x03), TAG, "mpu dlpf");
    ESP_RETURN_ON_ERROR(write_reg(mpu_dev, MPU9250_GYRO_CONFIG, IMU_GYRO_CONFIG_VALUE), TAG, "gyro 2000 dps");
    ESP_RETURN_ON_ERROR(write_reg(mpu_dev, MPU9250_ACCEL_CONFIG, 0x00), TAG, "accel 2g");
    ESP_RETURN_ON_ERROR(write_reg(mpu_dev, MPU9250_ACCEL_CONFIG2, 0x03), TAG, "accel dlpf");
    ESP_RETURN_ON_ERROR(write_reg(mpu_dev, MPU9250_INT_PIN_CFG, 0x02), TAG, "i2c bypass");
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_RETURN_ON_ERROR(write_reg(mag_dev, AK8963_CNTL1, 0x00), TAG, "mag powerdown");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(write_reg(mag_dev, AK8963_CNTL1, 0x0F), TAG, "mag fuse rom");
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t asa[3] = {0};
    ESP_RETURN_ON_ERROR(read_regs(mag_dev, AK8963_ASAX, asa, sizeof(asa)), TAG, "mag asa");
    for (int i = 0; i < 3; ++i) {
        mag_adjust[i] = (((float)asa[i] - 128.0f) / 256.0f) + 1.0f;
    }

    ESP_RETURN_ON_ERROR(write_reg(mag_dev, AK8963_CNTL1, 0x00), TAG, "mag powerdown 2");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(write_reg(mag_dev, AK8963_CNTL1, 0x16), TAG, "mag continuous 100hz 16bit");

    return ESP_OK;
}

static bool read_imu_raw(float accel[3], float gyro[3], float mag[3], float mag_uncalibrated[3], bool *mag_valid)
{
    uint8_t data[14] = {0};
    float sensor_accel[3] = {0};
    float sensor_gyro[3] = {0};
    float robot_accel[3] = {0};
    float robot_gyro[3] = {0};

    if (read_regs(mpu_dev, MPU9250_ACCEL_XOUT_H, data, sizeof(data)) != ESP_OK) {
        return false;
    }

    sensor_accel[0] = ((float)be16(&data[0]) / 16384.0f) * 9.80665f;
    sensor_accel[1] = ((float)be16(&data[2]) / 16384.0f) * 9.80665f;
    sensor_accel[2] = ((float)be16(&data[4]) / 16384.0f) * 9.80665f;
    sensor_gyro[0] = (float)be16(&data[8]) / IMU_GYRO_LSB_PER_DPS;
    sensor_gyro[1] = (float)be16(&data[10]) / IMU_GYRO_LSB_PER_DPS;
    sensor_gyro[2] = (float)be16(&data[12]) / IMU_GYRO_LSB_PER_DPS;

    sensor_to_robot_frame(sensor_accel, robot_accel);
    sensor_to_robot_frame(sensor_gyro, robot_gyro);

    for (int i = 0; i < 3; ++i) {
        accel[i] = robot_accel[i] - accel_bias[i];
        gyro[i] = robot_gyro[i] - gyro_bias[i];
    }

    *mag_valid = false;

    uint8_t st1 = 0;
    if (read_regs(mag_dev, AK8963_ST1, &st1, 1) == ESP_OK && (st1 & 0x01)) {
        uint8_t mag_data[7] = {0};
        if (read_regs(mag_dev, AK8963_HXL, mag_data, sizeof(mag_data)) == ESP_OK && !(mag_data[6] & 0x08)) {
            float sensor_mag[3] = {
                (float)le16(&mag_data[0]) * 0.15f * mag_adjust[0],
                (float)le16(&mag_data[2]) * 0.15f * mag_adjust[1],
                (float)le16(&mag_data[4]) * 0.15f * mag_adjust[2],
            };
            float mpu_frame_mag[3] = {0};
            mag_sensor_to_mpu_frame(sensor_mag, mpu_frame_mag);
            sensor_to_robot_frame(mpu_frame_mag, mag_uncalibrated);
            for (int i = 0; i < 3; ++i) {
                mag[i] = (mag_uncalibrated[i] - mag_bias[i]) * mag_scale[i];
            }
            *mag_valid = true;
        }
    }

    return true;
}

static void stationary_calibration_update(const float accel[3], const float gyro[3])
{
    if (!accel_gyro_calibrating || esp_timer_get_time() > stationary_cal_end_us) {
        return;
    }

    for (int i = 0; i < 3; ++i) {
        accel_sum[i] += accel[i] + accel_bias[i];
        gyro_sum[i] += gyro[i] + gyro_bias[i];
    }
    stationary_samples++;
}

static void stationary_calibration_finish_if_needed(void)
{
    if (!accel_gyro_calibrating || esp_timer_get_time() <= stationary_cal_end_us) {
        return;
    }

    if (stationary_samples == 0) {
        stationary_samples = 1;
    }

    gyro_bias[0] = gyro_sum[0] / (float)stationary_samples;
    gyro_bias[1] = gyro_sum[1] / (float)stationary_samples;
    gyro_bias[2] = gyro_sum[2] / (float)stationary_samples;
    accel_bias[0] = accel_sum[0] / (float)stationary_samples;
    accel_bias[1] = accel_sum[1] / (float)stationary_samples;
    accel_bias[2] = (accel_sum[2] / (float)stationary_samples) - 9.80665f;

    accel_gyro_calibrating = false;
    ESP_LOGI(TAG,
             "Gyro/accel calibrado gyro_bias=[%.3f %.3f %.3f] accel_bias=[%.3f %.3f %.3f]",
             gyro_bias[0],
             gyro_bias[1],
             gyro_bias[2],
             accel_bias[0],
             accel_bias[1],
             accel_bias[2]);

    portENTER_CRITICAL(&state_mux);
    state.accel_gyro_calibrating = false;
    state.accel_gyro_calibrated = true;
    portEXIT_CRITICAL(&state_mux);

    if (start_mag_after_accel_gyro) {
        start_mag_after_accel_gyro = false;
        ESP_ERROR_CHECK_WITHOUT_ABORT(imu_start_mag_calibration(pending_mag_duration_ms));
    }
}

static void yaw_drift_calibration_update(const float gyro[3])
{
    if (!yaw_drift_calibrating || esp_timer_get_time() > yaw_drift_cal_end_us) {
        return;
    }

    yaw_drift_sum += gyro[2] + gyro_bias[2];
    yaw_drift_samples++;
}

static void yaw_drift_calibration_finish_if_needed(void)
{
    if (!yaw_drift_calibrating || esp_timer_get_time() <= yaw_drift_cal_end_us) {
        return;
    }

    if (yaw_drift_samples == 0) {
        yaw_drift_samples = 1;
    }

    gyro_bias[2] = yaw_drift_sum / (float)yaw_drift_samples;
    const int32_t yaw_bias_mdps = (int32_t)lroundf(gyro_bias[2] * 1000.0f);
    esp_err_t save_ret = save_i32_to_nvs(IMU_NVS_KEY_YAW_GYRO_BIAS_MDPS, yaw_bias_mdps);
    if (save_ret != ESP_OK) {
        ESP_LOGW(TAG, "Falha ao salvar drift yaw: %s", esp_err_to_name(save_ret));
    }
    yaw_drift_calibrating = false;

    portENTER_CRITICAL(&state_mux);
    state.yaw_drift_calibrating = false;
    state.yaw_drift_calibrated = save_ret == ESP_OK;
    portEXIT_CRITICAL(&state_mux);

    ESP_LOGI(TAG,
             "Drift yaw calibrado gyro_z_bias=%.4f dps samples=%lu save=%s",
             gyro_bias[2],
             (unsigned long)yaw_drift_samples,
             esp_err_to_name(save_ret));
}

static bool imu_is_stationary(const float accel[3], const float gyro[3])
{
    const float accel_norm = sqrtf((accel[0] * accel[0]) + (accel[1] * accel[1]) + (accel[2] * accel[2]));
    const float gyro_norm = sqrtf((gyro[0] * gyro[0]) + (gyro[1] * gyro[1]) + (gyro[2] * gyro[2]));

    return fabsf(accel_norm - 9.80665f) < IMU_STATIONARY_ACCEL_DELTA_MPS2 &&
           gyro_norm < IMU_STATIONARY_GYRO_THRESHOLD_DPS;
}

static void gyro_bias_auto_update(const float accel[3], const float gyro[3])
{
    if (accel_gyro_calibrating || yaw_drift_calibrating || !imu_is_stationary(accel, gyro)) {
        return;
    }

    for (int i = 0; i < 3; ++i) {
        gyro_bias[i] += gyro[i] * IMU_GYRO_BIAS_AUTO_ALPHA;
    }
}

static void mag_calibration_update(const float raw_mag[3])
{
    bool calibrating = false;
    const float field_norm = sqrtf((raw_mag[0] * raw_mag[0]) +
                                   (raw_mag[1] * raw_mag[1]) +
                                   (raw_mag[2] * raw_mag[2]));

    portENTER_CRITICAL(&state_mux);
    calibrating = state.mag_calibrating;
    portEXIT_CRITICAL(&state_mux);

    if (!calibrating) {
        return;
    }

    if (field_norm < IMU_MAG_FIELD_MIN_UT || field_norm > IMU_MAG_FIELD_MAX_UT) {
        return;
    }

    for (int i = 0; i < 3; ++i) {
        if (raw_mag[i] < mag_min[i]) {
            mag_min[i] = raw_mag[i];
        }
        if (raw_mag[i] > mag_max[i]) {
            mag_max[i] = raw_mag[i];
        }
    }
    mag_cal_samples++;

    if (esp_timer_get_time() < mag_cal_end_us) {
        return;
    }

    float radius[3] = {0};
    float new_bias[3] = {0};
    float new_scale[3] = {1.0f, 1.0f, 1.0f};
    float avg_radius = 0.0f;
    bool calibration_valid = mag_cal_samples >= IMU_MAG_CALIBRATION_MIN_SAMPLES;
    for (int i = 0; i < 3; ++i) {
        const float span = mag_max[i] - mag_min[i];
        if (span < IMU_MAG_CALIBRATION_MIN_AXIS_SPAN_UT) {
            calibration_valid = false;
        }
        new_bias[i] = (mag_max[i] + mag_min[i]) * 0.5f;
        radius[i] = span * 0.5f;
        avg_radius += radius[i];
    }
    avg_radius /= 3.0f;

    if (!calibration_valid || avg_radius <= 0.0f) {
        portENTER_CRITICAL(&state_mux);
        state.mag_calibrating = false;
        state.mag_calibrated = false;
        portEXIT_CRITICAL(&state_mux);

        ESP_LOGW(TAG,
                 "Mag calibracao rejeitada samples=%lu span=[%.2f %.2f %.2f]",
                 (unsigned long)mag_cal_samples,
                 mag_max[0] - mag_min[0],
                 mag_max[1] - mag_min[1],
                 mag_max[2] - mag_min[2]);
        return;
    }

    for (int i = 0; i < 3; ++i) {
        new_scale[i] = radius[i] > 0.0f ? avg_radius / radius[i] : 1.0f;
        mag_bias[i] = new_bias[i];
        mag_scale[i] = new_scale[i];
    }

    portENTER_CRITICAL(&state_mux);
    state.mag_calibrating = false;
    state.mag_calibrated = true;
    portEXIT_CRITICAL(&state_mux);

    ESP_LOGI(TAG,
             "Mag calibrado samples=%lu bias=[%.2f %.2f %.2f] scale=[%.3f %.3f %.3f]",
             (unsigned long)mag_cal_samples,
             mag_bias[0],
             mag_bias[1],
             mag_bias[2],
             mag_scale[0],
             mag_scale[1],
             mag_scale[2]);
}

static void update_orientation(imu_state_t *next, float dt_s, bool use_mag_heading)
{
    const float ax = next->accel_mps2[0];
    const float ay = next->accel_mps2[1];
    const float az = next->accel_mps2[2];

    const float accel_roll = IMU_ROLL_SIGN * atan2f(ay, az) * DEG_PER_RAD;
    const float accel_pitch = IMU_PITCH_SIGN * atan2f(-ax, sqrtf((ay * ay) + (az * az))) * DEG_PER_RAD;

    float roll = next->roll_deg + (IMU_ROLL_SIGN * next->gyro_dps[0] * dt_s);
    float pitch = next->pitch_deg + (IMU_PITCH_SIGN * next->gyro_dps[1] * dt_s);
    const float yaw_rate_dps = next->gyro_dps[2];
    float yaw = next->yaw_deg + yaw_rate_dps * dt_s;

    roll = (IMU_COMPLEMENTARY_ALPHA * roll) + ((1.0f - IMU_COMPLEMENTARY_ALPHA) * accel_roll);
    pitch = (IMU_COMPLEMENTARY_ALPHA * pitch) + ((1.0f - IMU_COMPLEMENTARY_ALPHA) * accel_pitch);

    const float roll_rad = roll * RAD_PER_DEG;
    const float pitch_rad = pitch * RAD_PER_DEG;
    const float mx = next->mag_ut[0];
    const float my = next->mag_ut[1];
    const float mz = next->mag_ut[2];
    const float mag_norm = sqrtf((mx * mx) + (my * my) + (mz * mz));
    const float tilt_xh = mx * cosf(pitch_rad) + mz * sinf(pitch_rad);
    const float tilt_yh = (mx * sinf(roll_rad) * sinf(pitch_rad)) + (my * cosf(roll_rad)) -
                          (mz * sinf(roll_rad) * cosf(pitch_rad));
    const float mag_yaw_xy = mag_heading_from_plane(mx, my);
    const float mag_yaw_xz = mag_heading_from_plane(mx, mz);
    const float mag_yaw_yz = mag_heading_from_plane(my, mz);
    float selected_mag_yaw = mag_yaw_xz;
    float selected_axis_a = mx;
    float selected_axis_b = mz;

    switch (mag_heading_mode) {
    case IMU_MAG_HEADING_MODE_XY:
        selected_mag_yaw = mag_yaw_xy;
        selected_axis_a = mx;
        selected_axis_b = my;
        break;
    case IMU_MAG_HEADING_MODE_YZ:
        selected_mag_yaw = mag_yaw_yz;
        selected_axis_a = my;
        selected_axis_b = mz;
        break;
    case IMU_MAG_HEADING_MODE_XZ:
    default:
        selected_mag_yaw = mag_yaw_xz;
        selected_axis_a = mx;
        selected_axis_b = mz;
        break;
    }

    next->mag_field_norm_ut = mag_norm;
    next->mag_filter_gain = mag_yaw_correction_gain;
    next->yaw_drift_threshold_dps = yaw_drift_threshold_dps;
    next->mag_heading_mode = mag_heading_mode;
    next->mag_ignored = mag_ignored;
    next->mag_yaw_xy_deg = mag_yaw_xy;
    next->mag_yaw_xz_deg = mag_yaw_xz;
    next->mag_yaw_yz_deg = mag_yaw_yz;
    next->mag_yaw_error_deg = 0.0f;
    if (use_mag_heading && (fabsf(selected_axis_a) + fabsf(selected_axis_b)) > 0.001f) {
        const float yaw_error = wrap_180(selected_mag_yaw - yaw);
        next->mag_yaw_deg = selected_mag_yaw;
        next->mag_yaw_error_deg = yaw_error;
        yaw = wrap_180(yaw + (mag_yaw_correction_gain * yaw_error));
    } else if ((fabsf(selected_axis_a) + fabsf(selected_axis_b)) > 0.001f) {
        next->mag_yaw_deg = selected_mag_yaw;
    } else if ((fabsf(tilt_xh) + fabsf(tilt_yh)) > 0.001f) {
        next->mag_yaw_deg = mag_heading_from_plane(tilt_xh, tilt_yh);
    }

    bool reset_yaw = false;
    portENTER_CRITICAL(&state_mux);
    reset_yaw = yaw_reset_requested;
    yaw_reset_requested = false;
    portEXIT_CRITICAL(&state_mux);

    if (reset_yaw) {
        yaw = 0.0f;
    }

    next->roll_deg = wrap_180(roll);
    next->pitch_deg = wrap_180(pitch);
    next->yaw_deg = wrap_180(yaw);
    update_quaternion_from_euler(next);
}

static void imu_task(void *arg)
{
    (void)arg;
    int64_t last_us = esp_timer_get_time();
    uint32_t loop_count = 0;
    int64_t loop_window_start_us = last_us;
    float sample_hz = 0.0f;
    imu_state_t next = {0};

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        float accel[3] = {0};
        float gyro[3] = {0};
        float mag[3] = {0};
        float mag_uncalibrated[3] = {0};
        bool mag_valid = false;
        const int64_t now_us = esp_timer_get_time();
        ++loop_count;
        const int64_t window_us = now_us - loop_window_start_us;
        if (window_us >= 1000000) {
            sample_hz = ((float)loop_count * 1000000.0f) / (float)window_us;
            loop_count = 0;
            loop_window_start_us = now_us;
        }
        const float dt_s = (float)(now_us - last_us) / 1000000.0f;
        last_us = now_us;

        if (read_imu_raw(accel, gyro, mag, mag_uncalibrated, &mag_valid)) {
            next.sample_hz = sample_hz;
            stationary_calibration_update(accel, gyro);
            stationary_calibration_finish_if_needed();
            yaw_drift_calibration_update(gyro);
            yaw_drift_calibration_finish_if_needed();
            gyro_bias_auto_update(accel, gyro);
            memcpy(next.accel_mps2, accel, sizeof(accel));
            memcpy(next.gyro_dps, gyro, sizeof(gyro));
            if (mag_valid) {
                memcpy(next.mag_ut, mag, sizeof(mag));
                mag_calibration_update(mag_uncalibrated);
            }

            portENTER_CRITICAL(&state_mux);
            const bool accel_gyro_calibrating_state = state.accel_gyro_calibrating;
            const bool accel_gyro_calibrated = state.accel_gyro_calibrated;
            const bool yaw_drift_calibrating_state = state.yaw_drift_calibrating;
            const bool yaw_drift_calibrated = state.yaw_drift_calibrated;
            const bool mag_calibrating = state.mag_calibrating;
            const bool mag_calibrated = state.mag_calibrated;
            const bool ignore_mag = state.mag_ignored;
            portEXIT_CRITICAL(&state_mux);

            update_orientation(&next, dt_s, !ignore_mag && mag_valid && mag_calibrated && !mag_calibrating);

            portENTER_CRITICAL(&state_mux);
            state = next;
            state.accel_gyro_calibrating = accel_gyro_calibrating_state;
            state.accel_gyro_calibrated = accel_gyro_calibrated;
            state.yaw_drift_calibrating = yaw_drift_calibrating_state;
            state.yaw_drift_calibrated = yaw_drift_calibrated;
            state.mag_calibrating = mag_calibrating;
            state.mag_calibrated = mag_calibrated;
            portEXIT_CRITICAL(&state_mux);
        } else {
            portENTER_CRITICAL(&state_mux);
            state.sample_hz = sample_hz;
            portEXIT_CRITICAL(&state_mux);
        }

    }
}

static void imu_timer_cb(void *arg)
{
    (void)arg;

    if (imu_task_handle != NULL) {
        xTaskNotifyGive(imu_task_handle);
    }
}

esp_err_t imu_init(void)
{
    if (initialized) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(imu_bus_init(), TAG, "bus init");
    ESP_RETURN_ON_ERROR(mpu9250_init(), TAG, "mpu9250 init");

    load_settings_from_nvs();

    portENTER_CRITICAL(&state_mux);
    state.mag_filter_gain = mag_yaw_correction_gain;
    state.yaw_drift_threshold_dps = yaw_drift_threshold_dps;
    state.mag_heading_mode = mag_heading_mode;
    state.mag_ignored = mag_ignored;
    portEXIT_CRITICAL(&state_mux);

    BaseType_t created = xTaskCreatePinnedToCore(imu_task,
                                                 "imu_task",
                                                 4096,
                                                 NULL,
                                                 IMU_TASK_PRIORITY,
                                                 &imu_task_handle,
                                                 IMU_TASK_CORE_ID);
    if (created != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    const esp_timer_create_args_t timer_args = {
        .callback = imu_timer_cb,
        .skip_unhandled_events = true,
        .name = "imu_500hz",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &imu_timer_handle), TAG, "imu timer create");
    ESP_RETURN_ON_ERROR(esp_timer_start_periodic(imu_timer_handle, IMU_SAMPLE_PERIOD_US), TAG, "imu timer start");

    initialized = true;
    ESP_LOGI(TAG,
             "MPU-9250 iniciado SDA=%d SCL=%d sample=%.1fHz gyro=+-2000dps core=%d",
             IMU_MPU9250_SDA_GPIO,
             IMU_MPU9250_SCL_GPIO,
             1000000.0f / (float)IMU_SAMPLE_PERIOD_US,
             IMU_TASK_CORE_ID);
    return ESP_OK;
}

static esp_err_t start_accel_gyro_calibration_internal(uint32_t duration_ms,
                                                       bool start_mag_after,
                                                       uint32_t mag_duration_ms)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (duration_ms == 0) {
        duration_ms = IMU_ACCEL_GYRO_CALIBRATION_DEFAULT_MS;
    }
    if (duration_ms < 1000) {
        duration_ms = IMU_ACCEL_GYRO_CALIBRATION_DEFAULT_MS;
    }

    memset(accel_sum, 0, sizeof(accel_sum));
    memset(gyro_sum, 0, sizeof(gyro_sum));
    stationary_samples = 0;
    stationary_cal_end_us = esp_timer_get_time() + ((int64_t)duration_ms * 1000);
    pending_mag_duration_ms = mag_duration_ms == 0 ? IMU_MAG_CALIBRATION_DEFAULT_MS : mag_duration_ms;
    start_mag_after_accel_gyro = start_mag_after;
    accel_gyro_calibrating = true;

    portENTER_CRITICAL(&state_mux);
    state.accel_gyro_calibrating = true;
    state.accel_gyro_calibrated = false;
    if (start_mag_after) {
        state.mag_calibrating = false;
        state.mag_calibrated = false;
    }
    portEXIT_CRITICAL(&state_mux);

    ESP_LOGI(TAG,
             "Calibracao accel/gyro iniciada por %lu ms%s",
             (unsigned long)duration_ms,
             start_mag_after ? " com mag em seguida" : "");
    return ESP_OK;
}

esp_err_t imu_start_accel_gyro_calibration(uint32_t duration_ms)
{
    return start_accel_gyro_calibration_internal(duration_ms, false, 0);
}

esp_err_t imu_start_yaw_drift_calibration(uint32_t duration_ms)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (accel_gyro_calibrating) {
        return ESP_ERR_INVALID_STATE;
    }
    if (duration_ms == 0) {
        duration_ms = IMU_YAW_DRIFT_CALIBRATION_DEFAULT_MS;
    }
    if (duration_ms < 1000) {
        duration_ms = IMU_YAW_DRIFT_CALIBRATION_DEFAULT_MS;
    }

    yaw_drift_sum = 0.0f;
    yaw_drift_samples = 0;
    yaw_drift_cal_end_us = esp_timer_get_time() + ((int64_t)duration_ms * 1000);
    yaw_drift_calibrating = true;

    portENTER_CRITICAL(&state_mux);
    state.yaw_drift_calibrating = true;
    state.yaw_drift_calibrated = false;
    portEXIT_CRITICAL(&state_mux);

    ESP_LOGI(TAG, "Calibracao drift yaw iniciada por %lu ms", (unsigned long)duration_ms);
    return ESP_OK;
}

esp_err_t imu_start_mag_calibration(uint32_t duration_ms)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (duration_ms == 0) {
        duration_ms = IMU_MAG_CALIBRATION_DEFAULT_MS;
    }

    for (int i = 0; i < 3; ++i) {
        mag_min[i] = 1000000.0f;
        mag_max[i] = -1000000.0f;
    }
    mag_cal_samples = 0;
    mag_cal_end_us = esp_timer_get_time() + ((int64_t)duration_ms * 1000);

    portENTER_CRITICAL(&state_mux);
    state.mag_calibrating = true;
    state.mag_calibrated = false;
    portEXIT_CRITICAL(&state_mux);

    ESP_LOGI(TAG, "Calibracao magnetometro iniciada por %lu ms", (unsigned long)duration_ms);
    return ESP_OK;
}

esp_err_t imu_start_full_calibration(uint32_t duration_ms)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (duration_ms < 15000) {
        duration_ms = 45000;
    }

    const uint32_t accel_gyro_ms = IMU_ACCEL_GYRO_CALIBRATION_DEFAULT_MS;
    const uint32_t mag_duration_ms = duration_ms > accel_gyro_ms
                                         ? duration_ms - accel_gyro_ms
                                         : IMU_MAG_CALIBRATION_DEFAULT_MS;

    return start_accel_gyro_calibration_internal(accel_gyro_ms, true, mag_duration_ms);
}

esp_err_t imu_set_mag_filter_gain(float gain)
{
    if (gain < IMU_MAG_YAW_CORRECTION_GAIN_MIN || gain > IMU_MAG_YAW_CORRECTION_GAIN_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    mag_yaw_correction_gain = gain;
    portENTER_CRITICAL(&state_mux);
    state.mag_filter_gain = gain;
    portEXIT_CRITICAL(&state_mux);
    ESP_LOGI(TAG, "Ganho do filtro magnetico ajustado para %.4f", gain);
    return ESP_OK;
}

float imu_get_mag_filter_gain(void)
{
    return mag_yaw_correction_gain;
}

esp_err_t imu_set_mag_heading_mode(uint8_t mode)
{
    if (mode > IMU_MAG_HEADING_MODE_YZ) {
        return ESP_ERR_INVALID_ARG;
    }

    mag_heading_mode = mode;
    portENTER_CRITICAL(&state_mux);
    state.mag_heading_mode = mode;
    portEXIT_CRITICAL(&state_mux);
    ESP_LOGI(TAG, "Modo do heading magnetico ajustado para %u", (unsigned int)mode);
    return ESP_OK;
}

uint8_t imu_get_mag_heading_mode(void)
{
    return mag_heading_mode;
}

esp_err_t imu_set_mag_ignored(bool ignored)
{
    mag_ignored = ignored;
    portENTER_CRITICAL(&state_mux);
    state.mag_ignored = ignored;
    portEXIT_CRITICAL(&state_mux);

    esp_err_t ret = save_u8_to_nvs(IMU_NVS_KEY_MAG_IGNORED, ignored ? 1 : 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Falha ao salvar ignorar mag: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "Correcao por magnetometro %s", ignored ? "ignorada" : "ativa");
    return ret;
}

bool imu_get_mag_ignored(void)
{
    return mag_ignored;
}

esp_err_t imu_set_yaw_drift_threshold(float threshold_dps)
{
    if (threshold_dps < IMU_YAW_DRIFT_THRESHOLD_MIN_DPS || threshold_dps > IMU_YAW_DRIFT_THRESHOLD_MAX_DPS) {
        return ESP_ERR_INVALID_ARG;
    }

    yaw_drift_threshold_dps = threshold_dps;
    portENTER_CRITICAL(&state_mux);
    state.yaw_drift_threshold_dps = threshold_dps;
    portEXIT_CRITICAL(&state_mux);

    const int32_t threshold_mdps = (int32_t)lroundf(threshold_dps * 1000.0f);
    esp_err_t ret = save_i32_to_nvs(IMU_NVS_KEY_YAW_THRESHOLD_MDPS, threshold_mdps);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Falha ao salvar threshold yaw: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "Threshold de drift yaw ajustado para %.3f dps", threshold_dps);
    return ret;
}

float imu_get_yaw_drift_threshold(void)
{
    return yaw_drift_threshold_dps;
}

esp_err_t imu_reset_yaw(void)
{
    if (!initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&state_mux);
    yaw_reset_requested = true;
    state.yaw_deg = 0.0f;
    state.mag_yaw_error_deg = 0.0f;
    update_quaternion_from_euler(&state);
    portEXIT_CRITICAL(&state_mux);

    ESP_LOGI(TAG, "Yaw da IMU resetado para 0");
    return ESP_OK;
}

bool imu_get_state(imu_state_t *out_state)
{
    if (out_state == NULL || !initialized) {
        return false;
    }

    portENTER_CRITICAL(&state_mux);
    *out_state = state;
    portEXIT_CRITICAL(&state_mux);
    return true;
}
