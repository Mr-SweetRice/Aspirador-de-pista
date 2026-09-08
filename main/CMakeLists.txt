idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES comms control imu motors encoder odometry memory line_sensor battery_level rgb_led safety portal_sensor nvs_flash
)
