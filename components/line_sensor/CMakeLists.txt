idf_component_register(
    SRCS "line_sensor.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_timer freertos log nvs_flash memory soc
)
