#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_https_ota.h"

#include "wifi.h"
#include "mqtt.h"
#include "sdcard.h"
#include "ota_update.h"

#define SDCARD_MOUNT_POINT "/sdcard"

static const char *TAG = "main";

void app_main(void){
    /* ***** INITIAL SETUP ***** */

    // NVS Initialization
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());

    // Event loop creation
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // WiFi initialization
    if(wifi_init_sta() == ESP_FAIL){
        ESP_LOGE(TAG, "Connection failed, reboot in 10 seconds");
        vTaskDelay(pdMS_TO_TICKS(10000));
        esp_restart();
    }

    // MQTT initialization
    // esp_mqtt_client_handle_t mqtt_client = mqtt_app_start();

    // SD card initialization
    // ESP_ERROR_CHECK(sdcard_init(SDCARD_MOUNT_POINT));

    // OTA update initialization
    esp_https_ota_config_t config = ota_init(CONFIG_OTA_HTTP_SERVER_URL);

    /* ***** SETUP COMPLETED ***** */

    /* ***** TASKS DECLARATION ***** */
}
