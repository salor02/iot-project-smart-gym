# Current Monitor Node

This directory contains an Arduino project for an **ESP8266** microcontroller, paired with an **INA219** current sensor via I2C.

## Purpose and Logic

This node's main objective is system validation. It independently measures the power consumption of the Vision Node (ESP32-CAM) and pushes the data over MQTT to the local broker for real-time visualization on the Node-RED dashboard.

## Validated States

The monitor validates three primary states of the ESP32-CAM:

1.  **Deep Sleep:** The Vision Node is asleep awaiting an interrupt. Power consumption is ~2mA.
2.  **Awake Idle:** The Vision Node is triggered awake, Wi-Fi is connected, but the HTTP server isn't actively streaming video. Power consumption rises to ~140mA.
3.  **Awake Streaming:** The Pose Extractor (Python script) is actively pulling MJPEG frames from the Vision Node. Power consumption peaks at ~220mA.

## Build

Built and uploaded using the Arduino IDE. Ensure you select generic ESP8266 as your board and configure the correct Wi-Fi/MQTT credentials before flashing.