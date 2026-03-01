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

#include "wifi.h"
#include "mqtt.h"
#include "env_sensors.h"
#include "pir.h"
#include "pms.h"

static const char *TAG = "main";

// this function waits for a data ready event to be raised and then it display the message through the serial monitor
void serial_logger_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data){
    if(base == PIR_EVENT){
        switch(id){
            case EVENT_MOTION_DETECTED: { 
                printf(" ----- HC-SR501 PIR data begin ----- \n");
                printf("Motion detected\n");
                printf(" ----- HC-SR501 PIR data end ----- \n");
                break;
            }

            case EVENT_MOTION_STOPPED: {
                printf(" ----- HC-SR501 PIR data begin ----- \n");
                printf("Motion stopped\n");
                printf(" ----- HC-SR501 PIR data end ----- \n");
                break;
            }

            case EVENT_MOTION_CONFIRMED: { 
                printf(" ----- HC-SR501 PIR data begin ----- \n");
                printf("Motion confirmed\n");
                printf(" ----- HC-SR501 PIR data end ----- \n");
                break;
            }

            case EVENT_MOTION_TIMEOUT: {
                printf(" ----- HC-SR501 PIR data begin ----- \n");
                printf("Motion timeout\n");
                printf(" ----- HC-SR501 PIR data end ----- \n");
                break;
            }
        }
        return;
    }

    if(base == PMS_EVENT){
        switch(id){
            case EVENT_PMS_DATA_READY: {
                pms_data_t *pms_data = (pms_data_t*) event_data;
                printf(" ----- PMS5003 data begin ----- \n");
                printf("pm10: %d ug/m3\n", pms_data->pm10);
                printf("pm2.5: %d ug/m3\n", pms_data->pm2_5);
                printf("pm1.0: %d ug/m3\n", pms_data->pm1_0);
                printf("particles > 0.3um / 0.1L: %d\n", pms_data->particles_03um);
                printf("particles > 0.5um / 0.1L: %d\n", pms_data->particles_05um);
                printf("particles > 1.0um / 0.1L: %d\n", pms_data->particles_10um);
                printf("particles > 2.5um / 0.1L: %d\n", pms_data->particles_25um);
                printf("particles > 5.0um / 0.1L: %d\n", pms_data->particles_50um);
                printf("particles > 10.0um / 0.1L: %d\n", pms_data->particles_100um);
                printf(" ----- PMS5003 data end ----- \n");
                break;
            }
        }
        return;
    }

    if (base == ENV_SENSORS_EVENT){
        switch (id) {
            case EVENT_ENV_DATA_READY: {
                env_data_t *env_data = (env_data_t*) event_data;
                printf(" ----- ENVIRONMENT data begin ----- \n");
                printf("Temperature: %.2f °C, Humidity: %.2f %%\n", env_data->temp, env_data->humidity);
                printf("MQ-7 (CO): %d | MQ-135 (Air): %d\n", env_data->mq7_val, env_data->mq135_val);
                printf(" ----- ENVIRONMENT data end ----- \n");
                break;
            }
        }
        return;
    }
}

// this function waits for a data ready event to be raised and then it sends the message through MQTT, in a topic based on the event type itself.
void mqtt_publish_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data){

    esp_mqtt_client_handle_t mqtt_client = (esp_mqtt_client_handle_t) handler_args;
    
    char json_buffer[128];
        
    // format the MQTT message based on the received message's type and then publish it
    if(base == PIR_EVENT){
        switch(id){
            case EVENT_MOTION_CONFIRMED: {
                snprintf(json_buffer, sizeof(json_buffer), "{\"motion\": true}");
                esp_mqtt_client_publish(mqtt_client, "sensors/pir", json_buffer, 0, 1, 0);
                break;
            }

            case EVENT_MOTION_TIMEOUT: {
                snprintf(json_buffer, sizeof(json_buffer), "{\"motion\": false}");
                esp_mqtt_client_publish(mqtt_client, "sensors/pir", json_buffer, 0, 1, 0);
                break;
            }
        }
        return;
    }

    if(base == PMS_EVENT){
        switch(id){
            case EVENT_PMS_DATA_READY: {
                pms_data_t *pms_data = (pms_data_t*) event_data;

                snprintf(json_buffer, sizeof(json_buffer), 
                    "{\"pm1_0\": %d, \"pm2_5\": %d, \"pm10\": %d}", 
                    pms_data->pm1_0, pms_data->pm2_5, pms_data->pm10);
                
                esp_mqtt_client_publish(mqtt_client, "sensors/air_quality", json_buffer, 0, 1, 0);
                break;
            }
        }
        return;
    }

    if(base == ENV_SENSORS_EVENT){
        switch (id) {
            case EVENT_ENV_DATA_READY: {
                env_data_t *env_data = (env_data_t*) event_data;

                snprintf(json_buffer, sizeof(json_buffer), 
                    "{\"temp\": %.2f, \"humidity\": %.2f, \"co\": %d, \"air\": %d}", 
                    env_data->temp, env_data->humidity, 
                    env_data->mq7_val, env_data->mq135_val);
                
                esp_mqtt_client_publish(mqtt_client, "sensors/environment", json_buffer, 0, 1, 0);
                break;
            }
        }
        return;
    }
}

// this task is called by another task and produces a sound from the buzzer. It deletes itself upon termination
void sound_task(void *pvParameters){
    gpio_reset_pin(CONFIG_BUZZER_GPIO);
    gpio_set_direction(CONFIG_BUZZER_GPIO, GPIO_MODE_OUTPUT);

    for(int i = 0; i < 3; i++){
        gpio_set_level(CONFIG_BUZZER_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(CONFIG_BUZZER_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    vTaskDelete(NULL);
}

void sound_led_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data){
    switch(id){
        case EVENT_MOTION_DETECTED: { 
            gpio_set_level(CONFIG_LED_GPIO, 1); 
            break;
        }

        case EVENT_MOTION_STOPPED: {
            gpio_set_level(CONFIG_LED_GPIO, 0); 
            break;
        }

        case EVENT_MOTION_CONFIRMED: { 
            // xTaskCreate(start_sound, "buzzer_start_sound", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
            break;
        }

        case EVENT_MOTION_TIMEOUT: {
            // xTaskCreate(start_sound, "buzzer_start_sound", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
            break;
        }
    }
    return;
}

void app_main()
{
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
    esp_mqtt_client_handle_t mqtt_client = mqtt_app_start();

    // events' handlers binding
    ESP_ERROR_CHECK(esp_event_handler_register(ESP_EVENT_ANY_BASE, ESP_EVENT_ANY_ID, serial_logger_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(ESP_EVENT_ANY_BASE, ESP_EVENT_ANY_ID, mqtt_publish_handler, (void*) mqtt_client));
    ESP_ERROR_CHECK(esp_event_handler_register(PIR_EVENT, ESP_EVENT_ANY_ID, sound_led_handler, NULL));

    // LED initialization
    ESP_ERROR_CHECK(gpio_reset_pin(CONFIG_LED_GPIO));
    ESP_ERROR_CHECK(gpio_set_direction(CONFIG_LED_GPIO, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(CONFIG_LED_GPIO, 0));

    // PIR sensor initialization
    ESP_ERROR_CHECK(pir_init(CONFIG_PIR_GPIO));

    // PMS sensor initialization
    ESP_ERROR_CHECK(pms_init(CONFIG_PMS_UART_PORT, CONFIG_PMS_TX_GPIO, CONFIG_PMS_RX_GPIO, CONFIG_PMS_SET_GPIO));

    // Environmental sensors initialization
    ESP_ERROR_CHECK(temp_sensor_init(CONFIG_SHT3X_ADDR, CONFIG_I2C_MASTER_SDA, CONFIG_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(gas_sensors_init(CONFIG_ADC_UNIT, CONFIG_MQ7_ADC_CHANNEL, CONFIG_MQ135_ADC_CHANNEL));

    /* ***** SETUP COMPLETED ***** */

    /* ***** TASKS DECLARATION ***** */

    xTaskCreate(env_sensors_task, "env_sensors_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(pms_task, "pms_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}