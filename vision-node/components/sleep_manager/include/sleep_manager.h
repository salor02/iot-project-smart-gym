#pragma once
#include "driver/gpio.h"

void deep_sleep_register_ext0_wakeup(void);
esp_err_t deep_sleep_init(void);
void deep_sleep_manager_task(void *pvParameters);
