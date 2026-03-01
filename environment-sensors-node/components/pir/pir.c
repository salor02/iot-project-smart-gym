#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"

#include "sdkconfig.h"

#include "pir.h"

ESP_EVENT_DEFINE_BASE(PIR_EVENT);

static const char *TAG = "pir";

// PIR interrupt callback. It sends a new event regarding the current PIR status, alongside with the current_time data
static void IRAM_ATTR pir_isr_handler(void* arg) {
    gpio_num_t gpio_number = (gpio_num_t) arg;
    int32_t current_time = (uint32_t) esp_timer_get_time() / 1000;
    bool pir_status = gpio_get_level(gpio_number);

    esp_err_t err = esp_event_isr_post(PIR_EVENT, pir_status ? EVENT_MOTION_DETECTED : EVENT_MOTION_STOPPED, &current_time, sizeof(current_time), NULL);
    if(err != ESP_OK){
        ESP_LOGW(TAG, "error sending the EVENT_MOTION_DETECTED/STOPPED");
    }
}

// function called at the termination of the inactivity timer, it sends the timeout event
void pir_inactivity_timer_cb(void* arg){    
    esp_err_t err = esp_event_post(PIR_EVENT, EVENT_MOTION_TIMEOUT, NULL, 0, 0);
    if(err != ESP_OK){
        ESP_LOGW(TAG, "error sending the EVENT_MOTION_TIMEOUT");
    }
}

/*
    This function is a software filter to avoid false positives produced by the PIR sensor. It's a FSM and do the following:
    - filter paused motion paused: first start = 0, accepts just the EVENT_MOTION_DETECTED, 
        on EVENT_MOTION_DETECTED -> filter active motion active
    - filter active motion active: first start > 0
        on EVENT_MOTION_STOPPED -> filter active motion paused
    - filter active motion paused: first start > 0
        on EVENT_MOTION_DETECTED -> filter active motion active
    
    On both filter active motion active AND filter active motion paused, a check is performed based on the configured activation time:
    - if the elapsed time is not sufficient, the check is ignored
    - else
        - if the activation ratio is sufficient -> sends EVENT_MOTION_CONFIRMED
    whether or not the activation ratio is sufficient the filter is cleared and goes back to filter paused motion paused
*/
void pir_events_filter_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data){
    pir_data_t *pir_data = (pir_data_t*) handler_args;
    int32_t current_time = *(int32_t*) event_data;
    int32_t elapsed_time = (current_time - pir_data -> first_start) / 1000;

    // filter active
    if(pir_data -> first_start > 0){
        // filter active motion active -> filter active motion paused
        if(id == EVENT_MOTION_STOPPED){
            pir_data -> total_duration += current_time - pir_data -> last_start;
            ESP_LOGI(TAG, "PIR total activation time in the last %ds: %ds", elapsed_time, (uint32_t) pir_data -> total_duration/1000);
        }

        // if a sufficient time has elapsed, the check can be performed
        if(elapsed_time > CONFIG_PIR_ACTIVATION_TIME){
            float activation_ratio = (float) (pir_data -> total_duration / 1000) / elapsed_time;
            ESP_LOGI(TAG, "PIR total activation ratio in the last %ds : %f%%", elapsed_time, activation_ratio * 100);
            // this is the filter's core: if an activation ratio has been reached sends EVENT_MOTION_CONFIRMED
            if(activation_ratio >= (float) CONFIG_PIR_ACTIVATION_RATIO_THRESHOLD / 100){
                ESP_LOGI(TAG, "PIR activation ratio threshold reached, motion confirmed");
                esp_err_t err = esp_event_post(PIR_EVENT, EVENT_MOTION_CONFIRMED, NULL, 0, 0);
                if(err != ESP_OK){
                    ESP_LOGW(TAG, "error sending the EVENT_MOTION_CONFIRMED");
                }
            }
            else{
                ESP_LOGI(TAG, "Maximum PIR activation time elapsed (%ds), threshold not reached (%d)", CONFIG_PIR_ACTIVATION_TIME, CONFIG_PIR_ACTIVATION_RATIO_THRESHOLD);
            }
            //clear the filter after the preivous check, ready for another check
            pir_data -> first_start = 0;
            pir_data -> last_start = 0;
            pir_data -> total_duration = 0;
        }
    }

    // filter paused motion paused OR filter active motion paused
    if(id == EVENT_MOTION_DETECTED){
        if(pir_data -> first_start == 0) pir_data -> first_start = current_time;
        pir_data -> last_start = current_time;   
    }
}

/*
    This function manages the renewal mechanism of the PIR sensor:
    - on first EVENT_MOTION_CONFIRMED a timer is started and the vision node is turned on
    - on the following EVENT_MOTION_CONFIRMED the timer is restarted
    - on the timer expiration (no motion confirmed for a certain configured timeout time) the callback will raise the EVENT_MOTION_TIMEOUT, captured by
      this handler that will shut down the vision node
*/
void pir_events_renewal_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data){
    pir_data_t *pir_data = (pir_data_t*) handler_args;

    // this event is sent by the pir filter
    if(id == EVENT_MOTION_CONFIRMED){
        uint64_t timeout_us = CONFIG_PIR_TIMEOUT_TIME * 1000000ULL; 

        // timer not active -> first motion confirmed
        if(!esp_timer_is_active(pir_data->inactivity_timer)){
            esp_timer_start_once(pir_data->inactivity_timer, timeout_us);
            ESP_LOGI(TAG, "Inactivity timer started, a confirmation is requested in the next %d seconds", CONFIG_PIR_TIMEOUT_TIME);
        }
        else{
            // timer already active -> timer is restarted
            esp_timer_restart(pir_data->inactivity_timer, timeout_us);
            ESP_LOGI(TAG, "Inactivity timer renewed, another confirmation is requested in the next %d seconds", CONFIG_PIR_TIMEOUT_TIME);
        }
    }

    // this event is sent by the inactivity timer callback
    if(id == EVENT_MOTION_TIMEOUT){
        ESP_LOGI(TAG, "No motion confirmed in the last %d seconds, timeout reached", CONFIG_PIR_TIMEOUT_TIME);
    }
}

esp_err_t pir_init(gpio_num_t gpio_num){
    // PIR interrupt configuration
    gpio_config_t pir_config = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,      
        .pull_down_en = GPIO_PULLDOWN_ENABLE, // reduces false positives
        .intr_type = GPIO_INTR_ANYEDGE       // an interrupt will be raised in rising edge (when PIR detect motion)
    };
    ESP_ERROR_CHECK(gpio_config(&pir_config));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(gpio_num, pir_isr_handler, (void*) gpio_num));

    // PIR status initialization, it is useful for the filtering and renewal mechanism
    pir_data_t *pir_data = calloc(1, sizeof(pir_data_t));

    // PIR inactivy timer creation
    const esp_timer_create_args_t inactivity_timer_args = {
        .callback = &pir_inactivity_timer_cb,
        .arg = pir_data,
        .name = "pir_inactivity"
    };
    ESP_ERROR_CHECK(esp_timer_create(&inactivity_timer_args, &pir_data->inactivity_timer));

    // PIR events binding
    ESP_ERROR_CHECK(esp_event_handler_register(PIR_EVENT, EVENT_MOTION_DETECTED, pir_events_filter_handler, (void*) pir_data));
    ESP_ERROR_CHECK(esp_event_handler_register(PIR_EVENT, EVENT_MOTION_STOPPED, pir_events_filter_handler, (void*) pir_data));
    ESP_ERROR_CHECK(esp_event_handler_register(PIR_EVENT, EVENT_MOTION_CONFIRMED, pir_events_renewal_handler, (void*) pir_data));
    ESP_ERROR_CHECK(esp_event_handler_register(PIR_EVENT, EVENT_MOTION_TIMEOUT, pir_events_renewal_handler, (void*) pir_data));

    return ESP_OK;
}