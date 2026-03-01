#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "sht3x.h"

#include "env_sensors.h"

ESP_EVENT_DEFINE_BASE(ENV_SENSORS_EVENT); 

static const char *TAG = "env_sensors";

static sht3x_t dev;
static adc_oneshot_unit_handle_t adc_handle;
static uint8_t MQ7_ADC_CHANNEL;
static uint8_t MQ135_ADC_CHANNEL;

esp_err_t temp_sensor_init(uint8_t addr, gpio_num_t sda_gpio, gpio_num_t scl_gpio){    
    // I2C dev initialization
    ESP_ERROR_CHECK(i2cdev_init());

    // SHT31 sensor initialization
    memset(&dev, 0, sizeof(sht3x_t));
    ESP_ERROR_CHECK(sht3x_init_desc(&dev, addr, 0, sda_gpio, scl_gpio));
    ESP_ERROR_CHECK(sht3x_init(&dev));

    return ESP_OK;
}

esp_err_t gas_sensors_init(int8_t adc_unit, uint8_t mq7_channel, uint8_t mq135_channel){
    MQ7_ADC_CHANNEL = mq7_channel;
    MQ135_ADC_CHANNEL = mq135_channel;

    // ADC unit initialization    
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = (adc_unit_t)(adc_unit - 1),
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
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, (adc_channel_t)MQ7_ADC_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, (adc_channel_t)MQ135_ADC_CHANNEL, &config));

    return ESP_OK;
}

// this task performs all the "fast" readings sequentially
void env_sensors_task(void *pvParameters){
    while (1) {
        env_data_t env_data;
        esp_err_t err;

        // SHT31 reading
        sht3x_measure(&dev, &env_data.temp, &env_data.humidity);

        // gas sensors reading (ADC)
        err = adc_oneshot_read(adc_handle, MQ7_ADC_CHANNEL, &env_data.mq7_val);
        if(err != ESP_OK){
            ESP_LOGW(TAG, "adc mq7 read failed: %s", esp_err_to_name(err));
            continue;
        }
        err = adc_oneshot_read(adc_handle, MQ135_ADC_CHANNEL, &env_data.mq135_val);
        if(err != ESP_OK){
            ESP_LOGW(TAG, "adc mq135 read failed: %s", esp_err_to_name(err));
            continue;
        }

        /*
            Raise the data ready event, if the event queue is full return an error. 
            Please note: if there was an error during the reading of the MQx sensors, this event will not be raised.
        */
        err = esp_event_post(ENV_SENSORS_EVENT, EVENT_ENV_DATA_READY, &env_data, sizeof(env_data), 0);
        if(err != ESP_OK){
            ESP_LOGW(TAG, "error sending the EVENT_ENV_DATA_READY");
            continue;
        }

        ESP_LOGI(TAG, "environment sensors EVENT_ENV_DATA_READY sent");
        
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}