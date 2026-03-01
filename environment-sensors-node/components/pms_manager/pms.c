#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"

#include "pms.h"

ESP_EVENT_DEFINE_BASE(PMS_EVENT);

static const char *TAG = "pms";

static uart_port_t UART_PORT;
static gpio_num_t TX_PIN;
static gpio_num_t RX_PIN;
static gpio_num_t SET_PIN;
static int START_BYTE;

esp_err_t pms_init(uart_port_t uart_port, gpio_num_t tx, gpio_num_t rx, gpio_num_t set){

    UART_PORT = uart_port;
    TX_PIN = tx;
    RX_PIN = rx;
    SET_PIN = set;

// UART driver initialization for the PMS sensor
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // it is expected only to receive data, this is the reason why the RX buffer is 2048 and the TX is 0
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 2048, 0, 0, NULL, 0));

    #ifdef CONFIG_INDOOR_MODE
        START_BYTE = 4;
    #else
        START_BYTE = 10;
    #endif

    return ESP_OK;
}

/*  this is a raw UART reader for the PMS5003, plese refer to the official datasheet for further information
    (https://www.digikey.com/en/htmldatasheets/production/2903006/0/0/1/pms5003-series-manual) */
void pms_task(void *pvParameters) {
    // set the SET pin mode, this pin allows to turn the sensor on and off
    ESP_ERROR_CHECK(gpio_reset_pin(SET_PIN));
    ESP_ERROR_CHECK(gpio_set_direction(SET_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(SET_PIN, 0));

    // message type for this function will always be PM
    pms_data_t pms_data;
    uint8_t data[32];

    while (1) {
        // wake up sensor and wait 30s (minimum delay recommended before a reading)
        ESP_LOGI(TAG, "waking up PMS sensor");
        gpio_set_level(SET_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(30000));
        
        // retry to read the sensor in case of a checksum error or header/length error, for a limited number of times
        for(int retry = 0; retry < CONFIG_PMS_MAX_RETRY; retry++){
            // flush the frame queue in order to read just the last produced frame (wait for 5s if it's not available yet)
            uart_flush(UART_PORT);
            int len = uart_read_bytes(UART_PORT, data, 32, pdMS_TO_TICKS(5000));
            
            // lenght and header check
            if (len == 32 && data[0] == 0x42 && data[1] == 0x4D) {

                // checksum (this sum shall be the same as the value contained in the last 2 bytes of the frame)
                uint16_t checksum = 0;
                for (int i = 0; i < 30; i++) checksum += data[i];
                
                // if checksum is correct, proceed to the actual reading
                if (checksum == (data[30] << 8 | data[31])) {

                    // see the PMS5003 datasheet to know the byte ordering 
                    pms_data = (pms_data_t) {
                        .pm1_0 = ((data[START_BYTE]<<8) + data[START_BYTE+1]),
                        .pm2_5 = ((data[START_BYTE+2]<<8) + data[START_BYTE+3]),
                        .pm10 = ((data[START_BYTE+4]<<8) + data[START_BYTE+5]),
                        .particles_03um = ((data[16]<<8) + data[17]),
                        .particles_05um = ((data[18]<<8) + data[19]), 
                        .particles_10um = ((data[20]<<8) + data[21]), 
                        .particles_25um = ((data[22]<<8) + data[23]), 
                        .particles_50um = ((data[24]<<8) + data[25]), 
                        .particles_100um = ((data[26]<<8) + data[27])
                    };

                    // raise the data ready event, if the event queue is full return an error
                    esp_err_t err = esp_event_post(PMS_EVENT, EVENT_PMS_DATA_READY, &pms_data, sizeof(pms_data), 0);
                    if(err != ESP_OK){
                        ESP_LOGW(TAG, "error sending the EVENT_PMS_DATA_READY");
                    }
                    break;

                } else {
                    ESP_LOGW(TAG, "Checksum error");
                }
            } else {
                ESP_LOGW(TAG, "Error in frame reading (frame's header or lenght is wrong), (attempt n. %d)", retry);
            }
        }

        // sleep sensor, the next reading will be in 30 seconds
        ESP_LOGI(TAG, "turning off PMS sensor");
        gpio_set_level(SET_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}