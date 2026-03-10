import threading
import queue
import os
import json
from datetime import datetime
import cv2
import numpy as np
import paho.mqtt.client as mqtt
import mediapipe as mp
from urllib.request import urlopen
from exerciseTracker import ExerciseTracker

BaseOptions = mp.tasks.BaseOptions
PoseLandmarker = mp.tasks.vision.PoseLandmarker
PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode

# Configuration
STREAM_URL = "http://192.168.1.52/stream"
MQTT_BROKER = "192.168.1.253"
MQTT_PORT = 1883
MQTT_TOPIC = "vision/cmd"
MODEL_PATH = "pose_landmarker_full.task"
OUTPUT_DIR = "recordings"
OUTPUT_FPS = 15

# Indices of the 13 joints we care about
JOINT_INDICES = {
    11: "left_shoulder",
    12: "right_shoulder",
    13: "left_elbow",
    14: "right_elbow",
    15: "left_wrist",
    16: "right_wrist",
    23: "left_hip",
    24: "right_hip",
    25: "left_knee",
    26: "right_knee",
    27: "left_ankle",
    28: "right_ankle",
}

# Module state
stop_event = threading.Event()
is_running = False
frame_queue = queue.Queue()
display_queue = queue.Queue(maxsize=2)
mqtt_client = None

# Run pose detection on a BGR frame and return the 13-joint dict or None
def extract_joints(landmarker, frame):
    # Color conversion BGR to RGB, openCV works with BGR and mediapipe with RGB
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)

    # Landmarks extraction
    result = landmarker.detect(mp_image)

    if not result.pose_landmarks:
        return None

    # Just the first person is considered here, since the use case requires the presence of only one person
    landmarks = result.pose_landmarks[0] 
    joints = {}
    for idx, name in JOINT_INDICES.items():
        lm = landmarks[idx]
        joints[name] = {
            "x": round(lm.x, 4),
            "y": round(lm.y, 4),
            "z": round(lm.z, 4),
            "visibility": round(lm.visibility, 3),
        }
    
    return joints

# ----- PRODUCER -----
# Read raw bytes from the ESP32-CAM MJPEG stream, extract JPEG frames
# by looking for SOI (0xFFD8) and EOI (0xFFD9) markers, and put them
# into the shared queue
def frame_producer():
    global is_running

    print("[Producer] Connecting to stream:", STREAM_URL)
    try:
        stream = urlopen(STREAM_URL)
    except Exception as e:
        print(f"[Producer] Failed to connect: {e}")
        is_running = False
        stop_event.set()
        return

    buf = b""
    try:
        # While the MQTT message "stop_rec" is not received
        while not stop_event.is_set():
            chunk = stream.read(4096)
            if not chunk:
                print("[Producer] Stream ended")
                break
            buf += chunk

            # Extract complete JPEG frames from the buffer, each JPEG frame starts with ffd8 and ends with ffd9. More than one frame can be inside one chunk
            while True:
                soi = buf.find(b"\xff\xd8")
                if soi == -1:
                    break
                eoi = buf.find(b"\xff\xd9", soi)
                if eoi == -1:
                    break

                # Copy each byte included in the jpeg in a frame from the reading buffer
                jpeg_bytes = buf[soi : eoi + 2]

                # Clear the frame just extracted from the buffer
                buf = buf[eoi + 2 :]

                # Enqueue the extracted frame
                frame_queue.put(jpeg_bytes)
    except Exception as e:
        if not stop_event.is_set():
            print(f"[Producer] Error reading stream: {e}")
    finally:
        # Disconnect from the streaming server (ESP32 CAM)
        stream.close()
        print("[Producer] Stream closed")

# ----- CONSUMER -----
# Take JPEG frames from the queue, decode them, run pose detection,
# display them, and call the output hook
def frame_consumer():
    # Configure the landmark extraction. The image mode is used here since the 
    # strict real-time processing is not required and other modes would require a timestamp calculation,
    # which would be not strictly linear due to the frequent frame drop of the ESP32-CAM.
    options = PoseLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=MODEL_PATH),
        running_mode=VisionRunningMode.IMAGE,
    )

    exercise = ExerciseTracker()
    prev_reps = 0
    prev_state = "idle"
    mqtt_client.publish("vision/tracker_state", exercise.state)

    with PoseLandmarker.create_from_options(options) as landmarker:
        global video_writer
        frame_index = 0

        # Create video writer for this session
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        video_path = os.path.join(OUTPUT_DIR, f"exercise-{timestamp}.mp4")
        video_writer = None 

        # While the MQTT message "stop_rec" is not received
        while not stop_event.is_set():
            try:
                jpeg_bytes = frame_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            # Decode JPEG bytes into a BGR numpy array
            arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                continue

            joints = extract_joints(landmarker, frame)
            if joints is not None:
                exercise.detect(joints)
                if exercise.reps > prev_reps:
                    prev_reps = exercise.reps
                    # Publish session current reps via MQTT
                    payload = json.dumps({"exercise": exercise.exercise.name, "reps": exercise.reps, "timestamp": datetime.now().strftime("%d/%m/%Y %H:%M:%S")})
                    mqtt_client.publish("vision/sessions", payload)
                    print(f"[Consumer] Published session current stats: {payload}")
                
                if exercise.state != prev_state:
                    payload = exercise.state
                    mqtt_client.publish("vision/tracker_state", payload)
                    print(f"[Consumer] Published tracker current state: {payload}")

                # Draw joints on frame for debug
                h, w, _ = frame.shape
                for name, j in joints.items():
                    cx, cy = int(j["x"] * w), int(j["y"] * h)
                    cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
                    cv2.putText(frame, name, (cx + 8, cy - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)

            # Draw exercise info on the top-left corner
            ex_name = exercise.exercise.name if exercise.exercise else "Unrecognized"
            info_text = f"State: {exercise.state} | Reps: {exercise.reps} | Ex: {ex_name}"
            cv2.putText(frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # # Initialize video writer on first frame in order to know the resolution
            # if video_writer is None:
            #     h, w = frame.shape[:2]
            #     fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            #     video_writer = cv2.VideoWriter(video_path, fourcc, OUTPUT_FPS, (w, h))
            #     print(f"[Consumer] Recording to {video_path}")

            # # Write annotated frame to video file
            # video_writer.write(frame)

            # Send annotated frame to main thread for display
            if not display_queue.full():
                display_queue.put(frame)

            frame_index += 1

    # Release video writer when session ends
    if video_writer is not None:
        video_writer.release()
        video_writer = None
        print(f"[Consumer] Video saved to {video_path}")

    # Publish session summary via MQTT if the exercise is None (useful for debug purpose)
    if exercise.exercise is None:
        payload = json.dumps({"exercise": "None", "reps": exercise.reps, "timestamp": datetime.now().strftime("%d/%m/%Y %H:%M:%S")})
        mqtt_client.publish("vision/sessions", payload)
        print(f"[Consumer] Published session result for an invalid session: {payload}")

    # Signal that tracker has been stopped too
    mqtt_client.publish("vision/tracker_state", "stopped")
    print("[Consumer] Stopped")


# ----- SESSION MANAGEMENT -----
# Start the producer and consumer threads
def start_session():
    global is_running

    if is_running:
        print("[Session] Already running, ignoring start")
        return

    is_running = True
    stop_event.clear()
    # Drain any leftover frames from a previous session
    while not frame_queue.empty():
        frame_queue.get_nowait()

    # Producer and consumer threads
    print("[Session] Starting")
    producer = threading.Thread(target=frame_producer, daemon=True)
    consumer = threading.Thread(target=frame_consumer, daemon=True)
    producer.start()
    consumer.start()

    # Wait for both threads to finish
    producer.join()
    consumer.join()
    is_running = False
    print("[Session] Ended")

# Signal both threads to stop
def stop_session():
    global is_running

    if not is_running:
        print("[Session] Not running, ignoring stop")
        return
    
    print("[Session] Stopping")
    stop_event.set()

# ---- MQTT CALLBACKS -----
def on_connect(client, userdata, flags, rc, properties):
    if rc == 0:
        print(f"[MQTT] Connected to {MQTT_BROKER}:{MQTT_PORT}")
        client.subscribe(MQTT_TOPIC)
        print(f"[MQTT] Subscribed to {MQTT_TOPIC}")
    else:
        print(f"[MQTT] Connection failed with code {rc}")

def on_message(client, userdata, msg, properties=None):
    payload = msg.payload.decode("utf-8", errors="ignore").strip().lower()
    print(f"[MQTT] Received '{payload}' on {msg.topic}")

    if payload == "start_rec":
        # Run session in a separate thread so the MQTT loop stays responsive
        threading.Thread(target=start_session, daemon=True).start()
    elif payload == "stop_rec":
        stop_session()

# ----- MAIN -----
def main():
    global mqtt_client
    # mqtt setup
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message
    
    mqtt_client = client

    # Create output directory for recordings
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print(f"[Main] Connecting to MQTT broker at {MQTT_BROKER}:{MQTT_PORT} ...")
    client.connect(MQTT_BROKER, MQTT_PORT)
    
    # Start MQTT background thread
    client.loop_start()

    # Display GUI for debug
    try:
        while True:
            try:
                frame = display_queue.get(timeout=0.05)
                cv2.imshow("ESP32-CAM", frame)
            except queue.Empty:
                pass

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("[Main] 'q' pressed, stopping session")
                stop_session()
    except KeyboardInterrupt:
        print("[Main] Interrupted")
        stop_session()
    finally:
        cv2.destroyAllWindows()
        client.loop_stop()
        client.disconnect()

# main thread -> MQTT loop_start(): handles MQTT messages and manage sessions, each session consists of two threads
# producer thread -> reads data from the HTTP stream and send the produced frame in a queue
# consumer thread -> reads frame from the queue and apply the MediaPipe pose landmarks extracion
if __name__ == "__main__":
    main()
