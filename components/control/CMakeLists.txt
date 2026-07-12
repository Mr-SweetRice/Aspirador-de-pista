idf_component_register(
    SRCS "control.c"
    INCLUDE_DIRS "include"
    REQUIRES battery_level line_sensor motors odometry memory freertos log nvs_flash rgb_led safety
)
