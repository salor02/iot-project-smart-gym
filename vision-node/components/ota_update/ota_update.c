#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_log.h"

#include "ota_update.h"

static const char *TAG = "ota";

void ota_request_update(void *pvParameters){
    ESP_LOGI(TAG, "Starting new firmware download...");

    esp_http_client_config_t http_config = {
        .url = CONFIG_OTA_HTTP_SERVER_URL, 
        .keep_alive_enable = true,
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &http_config,
    };

    // this function performs the download and flash automatically
    esp_err_t ret = esp_https_ota(&ota_config);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Update successfully downloaded! Reboot...");
        esp_restart();
    } else {
        ESP_LOGE(TAG, "There was an error during the update. The old firmware will be executed now");
    }

    vTaskDelete(NULL);
}
