idf_component_register(
    SRCS "env_sensors.c"
    INCLUDE_DIRS "include"
    REQUIRES sht3x
    PRIV_REQUIRES esp_event esp_adc utils
)