idf_component_register(
    SRCS "pms.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_event esp_driver_gpio esp_driver_uart
)
