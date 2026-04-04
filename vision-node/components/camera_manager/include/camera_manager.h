#pragma once
#include "sdkconfig.h"
#include "esp_camera.h"

#if CONFIG_CAMERA_BOARD_ESP32_CAM_AI_THINKER

// ESP32-CAM AI-Thinker pin map
#define CAM_BOARD_NAME "ESP32-CAM AI-Thinker"
#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27
#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22
#define CAM_SENSOR_VFLIP 0
#define CAM_SENSOR_HMIRROR 0

#elif CONFIG_CAMERA_BOARD_ESP32S3_EYE

// ESP32-S3-EYE pin map
#define CAM_BOARD_NAME "ESP32-S3-EYE"
#define CAM_PIN_PWDN -1
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK 15
#define CAM_PIN_SIOD 4
#define CAM_PIN_SIOC 5
#define CAM_PIN_D7 16
#define CAM_PIN_D6 17
#define CAM_PIN_D5 18
#define CAM_PIN_D4 12
#define CAM_PIN_D3 10
#define CAM_PIN_D2 8
#define CAM_PIN_D1 9
#define CAM_PIN_D0 11
#define CAM_PIN_VSYNC 6
#define CAM_PIN_HREF 7
#define CAM_PIN_PCLK 13
#define CAM_SENSOR_VFLIP 1
#define CAM_SENSOR_HMIRROR 0

#endif

esp_err_t camera_init();
