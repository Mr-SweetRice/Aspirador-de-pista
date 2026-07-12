idf_component_register(
    SRCS "battery_level.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_adc esp_driver_gpio freertos log
)
