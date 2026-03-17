#include <stdio.h>
#include "sleep_manager.h"
#include "esp_sleep.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "sleep_manager";

// useful to know the sleep task to notify on the interrupt activation
static TaskHandle_t sleep_task_handle = NULL;

// register the rtc gpio for the wakeup
void deep_sleep_register_ext0_wakeup(void)
{
    ESP_LOGI(TAG, "Enabling EXT0 wakeup on pin GPIO%d\n", CONFIG_WAKEUP_PIN);
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(CONFIG_WAKEUP_PIN, 1));

    // Configure pullup/downs via RTCIO to tie wakeup pins to inactive level during deepsleep.
    // EXT0 resides in the same power domain (RTC_PERIPH) as the RTC IO pullup/downs.
    // No need to keep that power domain explicitly, unlike EXT1.
    ESP_ERROR_CHECK(rtc_gpio_init(CONFIG_WAKEUP_PIN));
    ESP_ERROR_CHECK(rtc_gpio_pullup_dis(CONFIG_WAKEUP_PIN));
    ESP_ERROR_CHECK(rtc_gpio_pulldown_en(CONFIG_WAKEUP_PIN));
}

// this handler is called on the falling edge of the wakeup gpio, it notifies the sleep manager task
static void IRAM_ATTR wakeup_pin_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(sleep_task_handle != NULL){
        vTaskNotifyGiveFromISR(sleep_task_handle, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// this function starts the deep sleep mode, after being notified by the interrupt
void deep_sleep_manager_task(void *pvParameters){
    sleep_task_handle = xTaskGetCurrentTaskHandle();

    while (1) {
        // Check manually first in case it was already 0
        if (gpio_get_level(CONFIG_WAKEUP_PIN) == 0) {
            ESP_LOGI(TAG, "GPIO %d went to zero, entering deep sleep...", CONFIG_WAKEUP_PIN);
            deep_sleep_register_ext0_wakeup();
            esp_deep_sleep_start();
        }
        
        // Wait indefinitely for the interrupt notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

// deep sleep initialization
esp_err_t deep_sleep_init(void){
    rtc_gpio_deinit(CONFIG_WAKEUP_PIN);

    gpio_config_t wakeup_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << CONFIG_WAKEUP_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&wakeup_conf);

    // if the isr service has been already installed by another component
    esp_err_t isr_err = gpio_install_isr_service(0);
    if(isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        return isr_err;
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_WAKEUP_PIN, wakeup_pin_isr_handler, NULL));

    return ESP_OK;
}