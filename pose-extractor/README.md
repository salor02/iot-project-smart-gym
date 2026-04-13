# Pose Extractor

The Pose Extractor acts as the core computer vision intelligence of the system. Written in Python 3, it consumes the MJPEG stream provided by the Vision Node and applies Machine Learning to track user exercises.

## Architecture

To prevent tracking delays and frame dropping, the application uses a multithreaded architecture featuring a shared frame buffer:

*   **Producer Thread:** Continuously reads raw MJPEG byte chunks from the HTTP stream and decodes them.
*   **Consumer Thread:** Pulls frames from the shared buffer, applies the AI model, and processes the application logic.

```mermaid
graph LR
    stream[MJPEG<br>stream] -- raw bytes --> producer[Producer<br>thread]
    producer -- push JPEG --> buffer([Shared<br>frame buffer])
    buffer -- pull JPEG --> consumer[Consumer<br>thread]
    consumer -- raw --> mediapipe[MediaPipe<br>Pose]
    mediapipe -- processed --> consumer
    consumer -- annotated frame --> video[Video<br>storage]
    consumer -- new rep --> mqtt[MQTT<br>Broker]
```

## AI and Exercise Tracking

The application uses **MediaPipe Pose** to track human joint landmarks (shoulders, elbows, wrists, hips, knees, ankles). An internal Finite-State Machine (FSM) is driven by the geometric angles formed by these joints to successfully classify and count repetitions.

```mermaid
stateDiagram-v2
    idle: Idle
    su: Start Unknown
    end: End-E
    se: Start-E

    idle --> su : is_standing
    su --> end : match(E_end)
    end --> se : match(E_start)<br>reps++
    se --> end : match(E_end)
```

For instance, a Bicep Curl is evaluated based on the following start and end positions:

### Start Position
![Start Position](../assets/bicep-start-v2.png)

### End Position
![End Position](../assets/bicep-end-v2.png)

**Supported Exercises:**
*   Squat
*   Romanian Deadlift (RDL)
*   Lateral Raise
*   Bicep Curl

The application listens to the `start_rec` MQTT command to begin extracting frames, and publishes completed repetition counts back to the MQTT Broker in real-time. Additionally, it can record and store annotated videos locally.

## Setup

```bash
pip install -r requirements.txt
python pose-extractor.py
```
*(Alternatively, use `docker compose up -d --build`).*