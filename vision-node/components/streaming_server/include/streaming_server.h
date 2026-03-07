#pragma once
#include "esp_err.h"
#include "esp_http_server.h"

ESP_EVENT_DECLARE_BASE(STREAMING_SERVER_EVENTS);
enum {
    STREAMING_SERVER_RECORDING_START,
    STREAMING_SERVER_RECORDING_STOP
};


esp_err_t stream_server_init(void);