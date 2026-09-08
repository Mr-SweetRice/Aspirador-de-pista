idf_component_register(
    SRCS "motors.c"
    INCLUDE_DIRS "include"
    REQUIRES encoder esp_driver_gpio esp_driver_ledc safety nvs_flash memory
)
