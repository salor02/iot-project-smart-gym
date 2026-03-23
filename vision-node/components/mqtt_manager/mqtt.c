#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "mqtt_client.h"

#include "mqtt.h"

ESP_EVENT_DEFINE_BASE(MQTT_MANAGER_EVENTS);

static const char *TAG = "mqtt";

static esp_mqtt_client_handle_t client = NULL;
static mqtt_manager_config_t mqtt_manager_cfg;

// called everytime the mqtt client receives a new event
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data){

    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id){
        // handle the connection established event and subscribe to the given topics
        case MQTT_EVENT_CONNECTED:{
            ESP_LOGI(TAG, "MQTT connection established");

            for (int i = 0; i < mqtt_manager_cfg.num_topics; i++) {
                if (mqtt_manager_cfg.subscribe_topics[i] != NULL) {
                    esp_mqtt_client_subscribe(client, mqtt_manager_cfg.subscribe_topics[i], 0);
                    ESP_LOGI(TAG, "Subscribed to: %s", mqtt_manager_cfg.subscribe_topics[i]);
                }
            }
            break;
        }
        
        // when a new message is received, an event is raised containing the received message
        case MQTT_EVENT_DATA:{
            mqtt_msg_t msg = {0};
            snprintf(msg.topic, sizeof(msg.topic), "%.*s", event->topic_len, event->topic);
            snprintf(msg.payload, sizeof(msg.payload), "%.*s", event->data_len, event->data);
            ESP_LOGI(TAG, "Received msg from topic %s (%s), raising MANAGER EVENT", msg.topic, msg.payload);
            esp_event_post(MQTT_MANAGER_EVENTS, MQTT_EVENT_NEW_MESSAGE, &msg, sizeof(mqtt_msg_t), portMAX_DELAY);
            break;
        }

        default:
            break;
    }
}

// public a given message in a given payload
void mqtt_manager_publish(const char *topic, const char *payload) {
    if (client != NULL) {
        esp_mqtt_client_publish(client, topic, payload, 0, 1, 0);
    }
}

// mqtt client initialization
void mqtt_manager_init(const mqtt_manager_config_t *config){
    mqtt_manager_cfg = *config;

    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = config->broker_uri,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client); 
}