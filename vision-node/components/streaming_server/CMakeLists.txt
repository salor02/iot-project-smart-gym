idf_component_register(
    SRCS "streaming_server.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp32-camera esp_http_server esp_timer camera_manager
)
