#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_event.h"

typedef struct{
    uint64_t first_start;
    uint64_t last_start;
    uint64_t total_duration;
    esp_timer_handle_t inactivity_timer;
} pir_data_t;

ESP_EVENT_DECLARE_BASE(PIR_EVENT);
enum{
    EVENT_MOTION_DETECTED,
    EVENT_MOTION_STOPPED,
    EVENT_MOTION_CONFIRMED,
    EVENT_MOTION_TIMEOUT
};

esp_err_t pir_init(gpio_num_t gpio_num);

