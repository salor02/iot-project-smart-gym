#pragma once
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// message struct for environmental data
typedef struct{
    float temp;
    float humidity;
    int mq7_val;
    int mq135_val;
} env_data_t;

// message struct for PMS data
typedef struct{
	uint16_t pm1_0;
	uint16_t pm2_5;
	uint16_t pm10;
	uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
	uint8_t sensor_id;
} pms_data_t;

typedef struct{
    uint64_t first_start;
    uint64_t last_start;
    uint64_t total_duration;
} pir_data_t;

// events' family declaration
ESP_EVENT_DECLARE_BASE(SENSOR_EVENTS); 
enum {
    EVENT_ENV_DATA_READY,
    EVENT_PMS_DATA_READY,
    EVENT_MOTION_DETECTED,
    EVENT_MOTION_STOPPED,
    EVENT_PIR_MOTION_CONFIRMED
};