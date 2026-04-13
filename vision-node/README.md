# Vision Node

This directory contains the firmware for the **ESP32-CAM** microcontroller, responsible for video capturing. Developed using **ESP-IDF**, it is optimized for power efficiency.

## Operational Flow

The Vision Node operates in two primary states to preserve energy:

1.  **Deep Sleep:** The device remains asleep, drawing ~2mA of current, completely shutting off its Wi-Fi and camera peripherals.
2.  **Awake & Streaming:** The Environment Node triggers the `EXT0` hardware wake-up line. Once awake, the ESP32-CAM initializes its Wi-Fi connection and starts a lightweight HTTP server.

Because the ESP32-CAM lacks a hardware video encoder, it simulates a video stream by capturing continuous sequence of JPEG frames and serving them as a multipart MJPEG stream over the `/stream` HTTP endpoint. When the Environment Node drops the wake-up line LOW, the HTTP session is securely closed, state is dropped, and the Node returns to Deep Sleep.

## Build and Flash

This is a standard ESP-IDF project.

```bash
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```
