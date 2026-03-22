#include <Wire.h>
#include <Adafruit_INA219.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Configuration
#define WIFI_SSID       "***"
#define WIFI_PASSWORD   "***"

#define MQTT_BROKER     "192.168.1.253"
#define MQTT_PORT       1883
#define MQTT_CLIENT_ID  "ina219_node"
#define MQTT_TOPIC      "sensors/ina219"

#define SAMPLE_INTERVAL_MS 1000  // sample every 1 second
#define MOVING_AVG_SIZE     5     // average over last 5 samples

Adafruit_INA219 ina219;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Circular buffer for moving average
float samples[MOVING_AVG_SIZE] = {0};
int   bufferIndex = 0;       // points to the oldest slot (next to overwrite)
int   sampleCount = 0;       // how many samples collected so far (max MOVING_AVG_SIZE)

// Adds a new sample and returns the current moving average
float updateMovingAverage(float newSample) {
    samples[bufferIndex] = newSample;
    bufferIndex = (bufferIndex + 1) % MOVING_AVG_SIZE;  // advance and wrap around
    if (sampleCount < MOVING_AVG_SIZE) sampleCount++;   // ramp-up phase

    // Sum only the valid samples collected so far
    float sum = 0;
    for (int i = 0; i < sampleCount; i++) {
        sum += samples[i];
    }
    return sum / sampleCount;
}

void connectWifi() {
    Serial.printf("Connecting to %s", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\nWiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
}

void connectMqtt() {
    while (!mqttClient.connected()) {
        Serial.print("Connecting to MQTT broker...");
        if (mqttClient.connect(MQTT_CLIENT_ID)) {
            Serial.println("connected");
        } else {
            Serial.printf("failed (rc=%d), retrying in 5s\n", mqttClient.state());
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(9600);
    Wire.begin(4, 5);

    if (!ina219.begin(&Wire)) {
        Serial.println("Failed to find INA219 chip");
        while (1) { delay(10); }
    }

    connectWifi();
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
}

void loop() {
    if (!mqttClient.connected()) connectMqtt();
    mqttClient.loop();

    static unsigned long lastSample = 0;
    if (millis() - lastSample >= SAMPLE_INTERVAL_MS) {
        lastSample = millis();

        float raw = ina219.getCurrent_mA();
        float avg = updateMovingAverage(raw);

        // Publish only when the buffer is full (first 5 samples are ramp-up)
        if (sampleCount == MOVING_AVG_SIZE) {
            char payload[48];
            snprintf(payload, sizeof(payload), "{\"current_mA\":%.2f}", avg);
            mqttClient.publish(MQTT_TOPIC, payload);
            Serial.printf("Published avg: %s\n", payload);
        }
    }
}
