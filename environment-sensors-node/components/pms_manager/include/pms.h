#pragma once
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/uart.h"

ESP_EVENT_DECLARE_BASE(PMS_EVENT);
enum{
    EVENT_PMS_DATA_READY
};

// message struct for PMS data
typedef struct{
	uint16_t pm1_0;
	uint16_t pm2_5;
	uint16_t pm10;
	uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
	uint8_t sensor_id;
} pms_data_t;

esp_err_t pms_init(uart_port_t uart_port, gpio_num_t tx, gpio_num_t rx, gpio_num_t set);
void pms_task(void *pvParameters);