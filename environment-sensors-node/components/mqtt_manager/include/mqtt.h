#pragma once
#include "mqtt_client.h"
#include "esp_event.h"

ESP_EVENT_DECLARE_BASE(MQTT_MANAGER_EVENTS);

enum {
    MQTT_EVENT_NEW_MESSAGE,
};

typedef struct {
    char topic[64];
    char payload[128];
} mqtt_msg_t;

#define MAX_SUBSCRIBE_TOPICS 5

typedef struct {
    const char *broker_uri;
    const char *subscribe_topics[MAX_SUBSCRIBE_TOPICS];
    int num_topics;
} mqtt_manager_config_t;

void mqtt_manager_init(const mqtt_manager_config_t *config);
void mqtt_manager_publish(const char *topic, const char *payload);
