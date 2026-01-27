#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dht.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define DHT_SENSOR_TYPE DHT_TYPE_AM2301
#define DHT_GPIO 5
#define BUZZER_GPIO 4
#define PIR_GPIO 6

void mq7_test(){
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, // 12-bit (0-4095)
        .atten = ADC_ATTEN_DB_12,         // Attenuazione 11dB per leggere fino a ~3.1V
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_8, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_9, &config));

    int mq7_val, mq135_val;
    while(1){
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_8, &mq7_val));
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_9, &mq135_val));

        printf("MQ-7 (CO): %d | MQ-135 (Air): %d\n", mq7_val, mq135_val);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void dht_test(void *pvParameters)
{
    float temperature, humidity;

    while (1)
    {
        if (dht_read_float_data(DHT_SENSOR_TYPE, DHT_GPIO, &humidity, &temperature) == ESP_OK)
            printf("Humidity: %.1f%% Temp: %.1fC\n", humidity, temperature);
        else
            printf("Could not read data from sensor\n");

        vTaskDelay(pdMS_TO_TICKS(4000));
    }
}

void pir_test(){
    gpio_reset_pin(PIR_GPIO);
    gpio_set_direction(PIR_GPIO, GPIO_MODE_INPUT);

    int previous_state = 0;

    while (1) {
        int motion = gpio_get_level(PIR_GPIO);

        if(motion && !previous_state){
            printf("MOVIMENTO RILEVATO!\n");
            previous_state = !previous_state;
            continue;
        }
        
        if(!motion && previous_state){
            printf("MOVIMENTO STOP!\n\n");
            previous_state = !previous_state;
            continue;
        } 

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void start_sound(){
    gpio_reset_pin(BUZZER_GPIO);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);

    for(int i = 0; i < 3; i++){
        gpio_set_level(BUZZER_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(BUZZER_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    start_sound();
    xTaskCreate(dht_test, "dht_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    // xTaskCreate(pir_test, "pir_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    xTaskCreate(mq7_test, "mq7_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

}