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
#include "streaming_server.h"

#define SDCARD_MOUNT_POINT "/sdcard"

static const char *TAG = "main";

// this function handles the events raised by the MQTT manager on a received message
static void mqtt_recipient(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    mqtt_msg_t *msg = (mqtt_msg_t *)event_data;
    
    if (strcmp(msg->topic, "vision/ota") == 0) {
        if (strcmp(msg->payload, "start") == 0) {
            ESP_LOGI(TAG, "Received OTA command (%s)", msg->payload);
            xTaskCreate(&ota_request_update, "ota_update", configMINIMAL_STACK_SIZE*3, NULL, 5, NULL);
            mqtt_manager_publish("vision/ota", "update_started");
        }
    }
}

// this function handles the node events and send notification trough mqtt
static void mqtt_sender(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    if(base == STREAMING_SERVER_EVENTS){
        switch(id){
            case STREAMING_SERVER_RECORDING_START: {
                ESP_LOGI(TAG, "Recording started");
                mqtt_manager_publish("vision/status", "recording_started");
                break;
            }
            case STREAMING_SERVER_RECORDING_STOP: {
                ESP_LOGI(TAG, "Recording stopped");
                mqtt_manager_publish("vision/status", "recording_stopped");
                break;
            }
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

    // WiFi initialization
    if(wifi_init_sta() == ESP_FAIL){
        ESP_LOGE(TAG, "Connection failed, reboot in 10 seconds");
        vTaskDelay(pdMS_TO_TICKS(10000));
        esp_restart();
    }

    // MQTT initialization
    mqtt_manager_config_t mqtt_config = {
        .broker_uri = CONFIG_MQTT_BROKER_URI,
        .subscribe_topics = {
            "vision/cmd",
            "vision/ota"
        },
        .num_topics = 2
    };
    mqtt_manager_init(&mqtt_config);
    mqtt_manager_publish("vision/status", "started");

    // SD card initialization
    // ESP_ERROR_CHECK(sdcard_init(SDCARD_MOUNT_POINT));

    // Check on the psram initialization
    if (!esp_psram_is_initialized()) {
        ESP_LOGE(TAG, "PSRAM not found. Check configuration");
        return;
    }
    ESP_LOGI(TAG, "PSRAM available: %d bytes", esp_psram_get_size());

    // Camera initialization
    ESP_ERROR_CHECK(camera_init());

    // Streaming server initialization
    ESP_ERROR_CHECK(stream_server_init());

    // Events binding
    esp_event_handler_register(MQTT_MANAGER_EVENTS, MQTT_EVENT_NEW_MESSAGE, mqtt_recipient, NULL);
    esp_event_handler_register(STREAMING_SERVER_EVENTS, ESP_EVENT_ANY_ID, mqtt_sender, NULL);

    /* ***** SETUP COMPLETED ***** */

    /* ***** TASKS DECLARATION ***** */
}
