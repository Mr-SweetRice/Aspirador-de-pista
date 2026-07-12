idf_component_register(
    SRCS "rgb_led.c"
    INCLUDE_DIRS "include"
    REQUIRES battery_level safety esp_driver_rmt esp_timer freertos log nvs_flash memory
)
