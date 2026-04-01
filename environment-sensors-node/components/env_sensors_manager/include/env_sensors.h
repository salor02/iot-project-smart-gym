#pragma once
#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// message struct for environmental data
typedef struct{
    float temp;
    bool temp_valid;
    float humidity;
    bool humidity_valid;
    int mq7_val;
    bool mq7_valid;
    int mq135_val;
    bool mq135_valid;
} env_data_t;

// events' family declaration
ESP_EVENT_DECLARE_BASE(ENV_SENSORS_EVENT); 
enum {
    EVENT_ENV_DATA_READY
};

void env_sensors_task(void *pvParameters);
esp_err_t temp_sensor_init(uint8_t addr, gpio_num_t sda_gpio, gpio_num_t scl_gpio);
esp_err_t gas_sensors_init(int8_t adc_unit, uint8_t mq7_channel, uint8_t mq135_channel);
