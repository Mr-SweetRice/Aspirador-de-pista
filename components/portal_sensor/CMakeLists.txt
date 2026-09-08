idf_component_register(
    SRCS "portal_sensor.c"
    INCLUDE_DIRS "include"
    REQUIRES control esp_driver_gpio esp_timer freertos log
)
