idf_component_register(
    SRCS "mqtt.c"
    INCLUDE_DIRS "include"
    REQUIRES mqtt
    PRIV_REQUIRES esp_netif esp_event
)
