#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_adc/adc_oneshot.h"
#include "nvs_flash.h"
#include "wifi.h"
#include "esp_log.h"
#include "mqtt.h"
#include "mqtt_client.h"
#include "sht3x.h"
#include "ina219.h"

#include "sensor_types.h"

ESP_EVENT_DEFINE_BASE(SENSOR_EVENTS);

static const char *TAG = "main";

// this function waits for a data ready event to be raised and then it display the message through the serial monitor
void serial_logger_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data){
    switch (id) {
        case EVENT_ENV_DATA_READY: {
            env_data_t *env_data = (env_data_t*) event_data;
            printf(" ----- ENVIRONMENT data begin ----- \n");
            printf("Temperature: %.2f °C, Humidity: %.2f %%\n", env_data->temp, env_data->humidity);
            printf("MQ-7 (CO): %d | MQ-135 (Air): %d\n", env_data->mq7_val, env_data->mq135_val);
            printf(" ----- ENVIRONMENT data end ----- \n");
            break;
        }

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
}

// this function waits for a data ready event to be raised and then it sends the message through MQTT, in a topic based on the event type itself.
void mqtt_publish_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data){

    esp_mqtt_client_handle_t mqtt_client = (esp_mqtt_client_handle_t) handler_args;
    
    char json_buffer[128];
    int msg_id;
        
    // format the MQTT message based on the received message's type and then publish it
    switch (id) {
        case EVENT_ENV_DATA_READY: {
            env_data_t *env_data = (env_data_t*) event_data;

            snprintf(json_buffer, sizeof(json_buffer), 
                "{\"temp\": %.2f, \"humidity\": %.2f, \"co\": %d, \"air\": %d}", 
                env_data->temp, env_data->humidity, 
                env_data->mq7_val, env_data->mq135_val);
            
            msg_id = esp_mqtt_client_publish(mqtt_client, "sensors/environment", json_buffer, 0, 1, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        }

        case EVENT_PMS_DATA_READY: {
            pms_data_t *pms_data = (pms_data_t*) event_data;

            snprintf(json_buffer, sizeof(json_buffer), 
                "{\"pm1_0\": %d, \"pm2_5\": %d, \"pm10\": %d}", 
                pms_data->pm1_0, pms_data->pm2_5, pms_data->pm10);
            
            msg_id = esp_mqtt_client_publish(mqtt_client, "sensors/air_quality", json_buffer, 0, 1, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        }
    }
}

// this task performs all the "fast" readings sequentially
void env_sensors_task(void *pvParameters){

    // SHT31 sensor initialization
    static sht3x_t dev;
    memset(&dev, 0, sizeof(sht3x_t));
    ESP_ERROR_CHECK(sht3x_init_desc(&dev, CONFIG_SHT3X_ADDR, 0, CONFIG_I2C_MASTER_SDA, CONFIG_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(sht3x_init(&dev));

    // ADC unit initialization
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = (adc_unit_t)(CONFIG_ADC_UNIT - 1),
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_handle));

    /* 
        Signal attenuation in order to read the whole signal width.
        Please note that this allows to read up to 3.9V IN, which is greater than
        the maximum DHT OUT voltage (3.3V). Therefore, the read value will never reach the
        maximum value (4095 for a 12 bit bitwidth).
    */
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,         
    };

    // ADC channels initialization
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, (adc_channel_t)CONFIG_MQ7_ADC_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, (adc_channel_t)CONFIG_MQ135_ADC_CHANNEL, &config));


    env_data_t env_data;

    while (1) {
        // SHT31 reading
        sht3x_measure(&dev, &env_data.temp, &env_data.humidity);

        // gas sensors reading (ADC)
        adc_oneshot_read(adc_handle, CONFIG_MQ7_ADC_CHANNEL, &env_data.mq7_val);
        adc_oneshot_read(adc_handle, CONFIG_MQ135_ADC_CHANNEL, &env_data.mq135_val);

        // raise the data ready event, if the event queue is full return an error
        esp_event_post(SENSOR_EVENTS, EVENT_ENV_DATA_READY, &env_data, sizeof(env_data), 0);
        
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

/*  this is a raw UART reader for the PMS5003, plese refer to the official datasheet for further information
    (https://www.digikey.com/en/htmldatasheets/production/2903006/0/0/1/pms5003-series-manual) */
static void pms_task(void *arg) {

    // UART driver initialization for the PMS sensor
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(CONFIG_UART_PORT, &uart_config);
    uart_set_pin(CONFIG_UART_PORT, CONFIG_TX_GPIO, CONFIG_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    // it is expected only to receive data, this is the reason why the RX buffer is 2048 and the TX is 0
    uart_driver_install(CONFIG_UART_PORT, 2048, 0, 0, NULL, 0);

    #ifdef CONFIG_INDOOR_MODE
        int start_byte = 4;
    #else
        int start_byte = 10;
    #endif

    // set the SET pin mode, this pin allows to turn the sensor on and off
    gpio_reset_pin(CONFIG_SET_GPIO);
    gpio_set_direction(CONFIG_SET_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_SET_GPIO, 0);

    // message type for this function will always be PM
    pms_data_t pms_data;
    uint8_t data[32];

    while (1) {
        // wake up sensor and wait 30s (minimum delay recommended before a reading)
        ESP_LOGI(TAG, "waking up PMS sensor");
        gpio_set_level(CONFIG_SET_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(30000));
        
        // flush the frame queue in order to read just the last produced frame (wait for 5s if it's not available yet)
        uart_flush(CONFIG_UART_PORT);
        int len = uart_read_bytes(CONFIG_UART_PORT, data, 32, pdMS_TO_TICKS(5000));
        
        // lenght and header check
        if (len == 32 && data[0] == 0x42 && data[1] == 0x4D) {

            // checksum (this sum shall be the same as the value contained in the last 2 bytes of the frame)
            uint16_t checksum = 0;
            for (int i = 0; i < 30; i++) checksum += data[i];
            
            // if checksum is correct, proceed to the actual reading
            if (checksum == (data[30] << 8 | data[31])) {

                // see the PMS5003 datasheet to know the byte ordering 
                pms_data = (pms_data_t) {
                    .pm1_0 = ((data[start_byte]<<8) + data[start_byte+1]),
                    .pm2_5 = ((data[start_byte+2]<<8) + data[start_byte+3]),
                    .pm10 = ((data[start_byte+4]<<8) + data[start_byte+5]),
                    .particles_03um = ((data[16]<<8) + data[17]),
                    .particles_05um = ((data[18]<<8) + data[19]), 
                    .particles_10um = ((data[20]<<8) + data[21]), 
                    .particles_25um = ((data[22]<<8) + data[23]), 
                    .particles_50um = ((data[24]<<8) + data[25]), 
                    .particles_100um = ((data[26]<<8) + data[27])
                };

                // raise the data ready event, if the event queue is full return an error
                esp_event_post(SENSOR_EVENTS, EVENT_PMS_DATA_READY, &pms_data, sizeof(pms_data), 0);

            } else {
                ESP_LOGE(TAG, "Checksum error");
            }
        } else {
            ESP_LOGE(TAG, "Error in frame reading (frame's header or lenght is wrong)");
        }

        // sleep sensor, the next reading will be in 30 seconds
        ESP_LOGI(TAG, "turning off PMS sensor");
        gpio_set_level(CONFIG_SET_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}

void pir_task(void *pvParameters){
    gpio_reset_pin(CONFIG_PIR_GPIO);
    gpio_set_direction(CONFIG_PIR_GPIO, GPIO_MODE_INPUT);

    int previous_state = 0;

    while (1) {
        int motion = gpio_get_level(CONFIG_PIR_GPIO);

        if(motion && !previous_state){
            printf("Motion detected!\n");
            previous_state = !previous_state;
            continue;
        }
        
        if(!motion && previous_state){
            printf("Motion stopped!\n\n");
            previous_state = !previous_state;
            continue;
        } 

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void start_sound(void *pvParameters){
    gpio_reset_pin(CONFIG_BUZZER_GPIO);
    gpio_set_direction(CONFIG_BUZZER_GPIO, GPIO_MODE_OUTPUT);

    for(int i = 0; i < 3; i++){
        gpio_set_level(CONFIG_BUZZER_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(CONFIG_BUZZER_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
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

    // I2C dev initialization
    ESP_ERROR_CHECK(i2cdev_init());

    // MQTT initialization
    esp_mqtt_client_handle_t mqtt_client = mqtt_app_start();

    // events' handlers binding
    ESP_ERROR_CHECK(esp_event_handler_register(SENSOR_EVENTS, ESP_EVENT_ANY_ID, serial_logger_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(SENSOR_EVENTS, ESP_EVENT_ANY_ID, mqtt_publish_handler, (void*) mqtt_client));

    /* ***** SETUP COMPLETED ***** */

    /* ***** TASKS DECLARATION ***** */

    // start_sound();
    xTaskCreate(env_sensors_task, "env_sensors_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(pms_task, "pms_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    // xTaskCreate(pir_task, "pir_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}