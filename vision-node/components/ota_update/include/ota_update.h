#pragma once
#include "esp_https_ota.h"

esp_https_ota_config_t ota_init(char* url);
void ota_request_update(void* handler_args, esp_event_base_t event_base, int32_t event_id, void* event_data);
