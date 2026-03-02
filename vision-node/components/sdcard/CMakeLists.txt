idf_component_register(
    SRCS "sdcard.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES sdmmc fatfs esp_driver_sdmmc
)
