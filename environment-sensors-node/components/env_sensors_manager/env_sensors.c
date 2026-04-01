#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "sht3x.h"

#include <math.h>

#include "env_sensors.h"
#include "moving_avarage.h"
#include "sdkconfig.h"

ESP_EVENT_DEFINE_BASE(ENV_SENSORS_EVENT); 

static const char *TAG = "env_sensors";

static sht3x_t dev;
static adc_oneshot_unit_handle_t adc_handle;
static uint8_t MQ7_ADC_CHANNEL;
static uint8_t MQ135_ADC_CHANNEL;

// calibrated constants
#define MQ7_R0      0.87f   // Rs (24.00) / 27.5
#define MQ135_R0    11.11f  // Rs (40.00) / 3.6

// convert raw adc value to a ppm value
static int calculate_ppm(int raw_adc, float r0, float a, float b) {
    if (raw_adc == 0 || raw_adc >= 4095) return 0;

    // esp32 pin voltage
    float v_adc = ((float)raw_adc / 4095.0f) * 3.3f;
    
    // original vout tension from sensor
    float v_out = v_adc * 1.5f;
    if (v_out <= 0.01f || v_out >= 5.0f) return 0;
    
    // the used sensors have an intern resistance of 1k
    float rs = 1.0f * (5.0f - v_out) / v_out;
    
    // calibration logs
    // float r0_mq7_cal = rs / 27.5f;
    // float r0_mq135_cal = rs / 3.6f;
    // ESP_LOGI(TAG, "[calibration] - Vout: %.2fV | Rs: %.2f kOhm | R0_Mq7: %.2f | R0_Mq135: %.2f", v_out, rs, r0_mq7_cal, r0_mq135_cal);
    
    float ratio = rs / r0;
    
    /*
        get ppm throught regression formula
        mq7 (CO): a = 99.04, b = -1.518
        mq135 (Air): a = 110.47, b = -2.86
    */
    float ppm = a * pow(ratio, b);
    
    return (int)ppm;
}

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

    // moving avarage filter initialization for each sensor data type
    static float temp_buf[CONFIG_ENV_SAMPLE_SIZE];
    static ma_filter_t temp_filter;
    ma_init(&temp_filter, temp_buf, CONFIG_ENV_SAMPLE_SIZE);

    static float humidity_buf[CONFIG_ENV_SAMPLE_SIZE];
    static ma_filter_t humidity_filter;
    ma_init(&humidity_filter, humidity_buf, CONFIG_ENV_SAMPLE_SIZE);

    static float co_buf[CONFIG_ENV_SAMPLE_SIZE];
    static ma_filter_t co_filter;
    ma_init(&co_filter, co_buf, CONFIG_ENV_SAMPLE_SIZE);

    static float air_buf[CONFIG_ENV_SAMPLE_SIZE];
    static ma_filter_t air_filter;
    ma_init(&air_filter, air_buf, CONFIG_ENV_SAMPLE_SIZE);

    while (1) {
        env_data_t env_data_raw = {0};
        env_data_t env_data_filtered = {0};
        esp_err_t err;

        // SHT31 reading
        err = sht3x_measure(&dev, &env_data_raw.temp, &env_data_raw.humidity);
        if (err == ESP_OK) {
            env_data_raw.temp_valid = true;
            env_data_raw.humidity_valid = true;
        } else {
            ESP_LOGW(TAG, "sht3x read failed: %s", esp_err_to_name(err));
        }

        int mq7_raw = 0;
        int mq135_raw = 0;

        // gas sensors reading (ADC)
        err = adc_oneshot_read(adc_handle, MQ7_ADC_CHANNEL, &mq7_raw);
        if (err == ESP_OK) {
            env_data_raw.mq7_val = calculate_ppm(mq7_raw, MQ7_R0, 99.04f, -1.518f);
            env_data_raw.mq7_valid = true;
        } else {
            ESP_LOGW(TAG, "adc mq7 read failed: %s", esp_err_to_name(err));
        }

        err = adc_oneshot_read(adc_handle, MQ135_ADC_CHANNEL, &mq135_raw);
        if (err == ESP_OK) {
            env_data_raw.mq135_val = calculate_ppm(mq135_raw, MQ135_R0, 110.47f, -2.86f);
            env_data_raw.mq135_valid = true;
        } else {
            ESP_LOGW(TAG, "adc mq135 read failed: %s", esp_err_to_name(err));
        }

        // update the moving average only with valid samples
        if (env_data_raw.temp_valid) {
            env_data_filtered.temp = ma_update(&temp_filter, env_data_raw.temp);
            env_data_filtered.temp_valid = true;
        }
        if (env_data_raw.humidity_valid) {
            env_data_filtered.humidity = ma_update(&humidity_filter, env_data_raw.humidity);
            env_data_filtered.humidity_valid = true;
        }
        if (env_data_raw.mq7_valid) {
            env_data_filtered.mq7_val = (int)ma_update(&co_filter, (float)env_data_raw.mq7_val);
            env_data_filtered.mq7_valid = true;
        }
        if (env_data_raw.mq135_valid) {
            env_data_filtered.mq135_val = (int)ma_update(&air_filter, (float)env_data_raw.mq135_val);
            env_data_filtered.mq135_valid = true;
        }

        /*
            Raise the data ready event even if one or more sensor readings failed.
            Invalid fields are marked through the *_valid flags.
        */
        err = esp_event_post(ENV_SENSORS_EVENT, EVENT_ENV_DATA_READY, &env_data_filtered, sizeof(env_data_filtered), 0);
        if(err != ESP_OK){
            ESP_LOGW(TAG, "error sending the EVENT_ENV_DATA_READY");
            continue;
        }

        ESP_LOGI(TAG, "environment sensors EVENT_ENV_DATA_READY sent (temp=%d humidity=%d mq7=%d mq135=%d)",
            env_data_filtered.temp_valid,
            env_data_filtered.humidity_valid,
            env_data_filtered.mq7_valid,
            env_data_filtered.mq135_valid);
        
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
