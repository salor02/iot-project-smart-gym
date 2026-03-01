idf_component_register(
    SRCS "pir.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_event esp_timer esp_driver_gpio
)
