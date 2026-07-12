idf_component_register(
    SRCS "imu.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_i2c esp_timer log freertos nvs_flash memory
)
