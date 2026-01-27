#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dht.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "nvs_flash.h"
#include "wifi.h"
#include "esp_log.h"

#if defined(CONFIG_DHT_TYPE_DHT11)
#define DHT_SENSOR_TYPE DHT_TYPE_DHT11
#endif
#if defined(CONFIG_DHT_TYPE_AM2301)
#define DHT_SENSOR_TYPE DHT_TYPE_AM2301
#endif
#if defined(CONFIG_DHT_TYPE_SI7021)
#define DHT_SENSOR_TYPE DHT_TYPE_SI7021
#endif

static const char *TAG = "main";

void gas_sensors_task(){

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

    int mq7_val, mq135_val;

    while(1){
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, (adc_channel_t)CONFIG_MQ7_ADC_CHANNEL, &mq7_val));
        ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, (adc_channel_t)CONFIG_MQ135_ADC_CHANNEL, &mq135_val));

        printf("MQ-7 (CO): %d | MQ-135 (Air): %d\n", mq7_val, mq135_val);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void dht_task()
{
    float temperature, humidity;

    while (1)
    {
        if (dht_read_float_data(DHT_SENSOR_TYPE, CONFIG_DHT_GPIO, &humidity, &temperature) == ESP_OK)
            printf("Humidity: %.1f%% Temp: %.1fC\n", humidity, temperature);
        else
            printf("Could not read data from sensor\n");

        vTaskDelay(pdMS_TO_TICKS(4000));
    }
}

void pir_task(){
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

void start_sound(){
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

    // WiFi initialization
    if(wifi_init_sta() == ESP_FAIL){
        ESP_LOGE(TAG, "Connection failed, reboot in 10 seconds");
        vTaskDelay(pdMS_TO_TICKS(10000));
        esp_restart();
    }

    /* ***** SETUP COMPLETED ***** */

    /* ***** TASKS DECLARATION ***** */

    // start_sound();
    xTaskCreate(dht_task, "dht_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    // xTaskCreate(pir_task, "pir_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(gas_sensors_task, "gas_sensors_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

}