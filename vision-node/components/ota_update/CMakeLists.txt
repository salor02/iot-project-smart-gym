idf_component_register(
    SRCS "ota_update.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_https_ota esp_http_client 
)
