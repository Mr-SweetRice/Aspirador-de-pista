idf_component_register(
    SRCS "comms_ble.c"
    INCLUDE_DIRS "include"
    REQUIRES bt nvs_flash imu motors encoder odometry memory control line_sensor battery_level rgb_led safety esp_timer
)
