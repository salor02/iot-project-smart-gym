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
#include "esp_psram.h"

#include "wifi.h"
#include "mqtt.h"
#include "sdcard.h"
#include "ota_update.h"
#include "camera_manager.h"

#define SDCARD_MOUNT_POINT "/sdcard"

static const char *TAG = "main";

// this function handles the events raised by the MQTT manager
static void mqtt_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    mqtt_msg_t *msg = (mqtt_msg_t *)event_data;
    
    if (strcmp(msg->topic, "vision/ota") == 0) {
        if (strcmp(msg->payload, "start") == 0) {
            ESP_LOGI(TAG, "Received OTA command (%s)", msg->payload);
            xTaskCreate(&ota_request_update, "ota_update", configMINIMAL_STACK_SIZE*3, NULL, 5, NULL);
            mqtt_manager_publish("vision/ota", "update_started");
        }
    }

    if (strcmp(msg->topic, "vision/cmd") == 0) {
        if (strcmp(msg->payload, "start_rec") == 0) {
            ESP_LOGI(TAG, "Received command (%s)", msg->payload);
            mqtt_manager_publish("vision/status", "recording_started");
        }
        if (strcmp(msg->payload, "stop_rec") == 0) {
            ESP_LOGI(TAG, "Received command (%s)", msg->payload);
            mqtt_manager_publish("vision/status", "recording_stopped");
        }
    }
}

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

    // // WiFi initialization
    // if(wifi_init_sta() == ESP_FAIL){
    //     ESP_LOGE(TAG, "Connection failed, reboot in 10 seconds");
    //     vTaskDelay(pdMS_TO_TICKS(10000));
    //     esp_restart();
    // }

    // // MQTT initialization
    // mqtt_manager_config_t mqtt_config = {
    //     .broker_uri = CONFIG_MQTT_BROKER_URI,
    //     .subscribe_topics = {
    //         "vision/cmd",
    //         "vision/ota"
    //     },
    //     .num_topics = 2
    // };
    // mqtt_manager_init(&mqtt_config);
    // mqtt_manager_publish("vision/status", "started");

    // SD card initialization
    // ESP_ERROR_CHECK(sdcard_init(SDCARD_MOUNT_POINT));

    // Check on the psram initialization
    if (!esp_psram_is_initialized()) {
        ESP_LOGE("MAIN", "PSRAM non trovata! Controlla la configurazione.");
        return;
    }
    ESP_LOGI("MAIN", "PSRAM disponibile: %d bytes", esp_psram_get_size());
      
    // Camera initialization
    ESP_ERROR_CHECK(camera_init());

    // Events binding
    // esp_event_handler_register(MQTT_MANAGER_EVENTS, MQTT_EVENT_NEW_MESSAGE, mqtt_handler, NULL);


    /* ***** SETUP COMPLETED ***** */

    /* ***** TASKS DECLARATION ***** */
}
