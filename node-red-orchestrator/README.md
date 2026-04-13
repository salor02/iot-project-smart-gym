# Node-RED Orchestrator

This directory contains the **Node-RED** orchestration layer for the Smart Home Gym project. The provided `flows.json` defines the local dashboard, the MQTT-based command logic, and the sensor-fusion step that combines environmental telemetry with exercise tracking results.

## Dashboard and Logic

Node-RED acts as the central hub between the user interface, the MQTT broker, and the backend services. The workspace is organized into three flows:

*   **Environment Node Flow:** subscribes to the environmental topics, displays live gauges and charts, and computes the ambient heat index from temperature and humidity.
*   **Vision Node Flow:** exposes dashboard buttons for `start_rec`, `stop_rec`, and OTA triggering, shows the Vision Node and tracker status, and keeps a live table of the recorded exercises.
*   **Current Value Flow:** subscribes to the INA219 current measurements used to validate the Vision Node power states.

At runtime, the dashboard is exposed under `/dashboard`, with dedicated pages such as `/environment-sensors` and `/vision-node`.

## Stress Index Estimation

The exercise history table also shows a **Stress Index** computed inside the flow. This value blends the ambient **Heat Index** estimated through the **NOAA Rothfusz regression** with an exercise-dependent **MET** coefficient and a repetition-based fatigue multiplier:

```text
m_f = 1 + 0.5 * (sqrt(r) - 1)
SI  = (HI_C * MET_e * m_f) / 4.8
```

The current flow maps the supported exercises to the following MET values:

*   **Squat:** `5.0`
*   **Romanian Deadlift:** `4.5`
*   **Lateral Raise:** `2.5`
*   **Bicep Curl:** `2.8`

The resulting score is rounded and shown together with timestamp, exercise name, repetitions, and recording URL.

## Usage

Before running the orchestrator, ensure that:

*   `flows.json` is imported into your Node-RED instance, or configured as the runtime flows file;
*   the MQTT broker configured in the flow as `mosquitto:1883` is reachable by the Node-RED instance;
*   the dashboard widgets required by the `ui-*` nodes are installed;
*   the recording URL inside the `recorded exercises history` function is adapted if your Node-RED host/IP differs from `http://192.168.1.253:1880/recordings/`.
