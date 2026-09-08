idf_component_register(
    SRCS "safety.c"
    INCLUDE_DIRS "include"
    REQUIRES battery_level encoder imu line_sensor odometry esp_timer freertos log nvs_flash memory
)
