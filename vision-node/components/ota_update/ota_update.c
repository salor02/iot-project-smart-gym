#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_log.h"

#include "ota_update.h"

static const char *TAG = "ota";

// OTA configuration
esp_https_ota_config_t ota_init(char* url){
    esp_http_client_config_t config = {
        .url = url, 
        .keep_alive_enable = true,
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &config,
    };
    
    return ota_config;
}

void ota_request_update(void* handler_args, esp_event_base_t event_base, int32_t event_id, void* event_data){
    ESP_LOGI(TAG, "Starting new firmware download...");

    esp_https_ota_config_t ota_config = *(esp_https_ota_config_t*) handler_args;

    // this function performs the download and flash automatically
    esp_err_t ret = esp_https_ota(&ota_config);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Update successfully downloaded! Reboot...");
        esp_restart();
    } else {
        ESP_LOGE(TAG, "There was an error during the update. The old firmware will be executed now");
    }
}
