from flask import Flask, Response
from ultralytics import YOLO
import cv2
import time

app = Flask(__name__)

print("Loading model YOLOv8n...")
model = YOLO('yolov8n.pt')
print("Model siap.")

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

if not cap.isOpened():
    print("ERROR: Kamera tidak bisa dibuka. Cek koneksi USB webcam.")

# --- Optimasi frame skip (dari versi sebelumnya) ---
FRAME_SKIP = 2
frame_count = 0
last_tracks = []  # simpan hasil tracking terakhir: list of (x1,y1,x2,y2,conf,track_id)

# --- FPS counter ---
prev_time = time.time()
fps = 0.0

# --- State target locking ---
locked_id = None
last_seen_time = None
LOST_TIMEOUT = 2.0  # detik, sebelum target dianggap hilang


def generate_frames():
    global frame_count, last_tracks, prev_time, fps, locked_id, last_seen_time

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Gagal membaca frame dari kamera.")
            break

        frame_h, frame_w = frame.shape[:2]
        frame_center_x = frame_w // 2

        current_time = time.time()
        fps = 1.0 / (current_time - prev_time) if current_time != prev_time else fps
        prev_time = current_time

        # --- Jalankan tracking hanya tiap FRAME_SKIP+1 frame ---
        if frame_count % (FRAME_SKIP + 1) == 0:
            # model.track() otomatis memberi ID unik antar frame (ByteTrack)
            results = model.track(frame, persist=True, verbose=False,
                                   imgsz=320, conf=0.5, classes=[0])[0]  # classes=[0] -> hanya "person"

            last_tracks = []
            if results.boxes is not None and results.boxes.id is not None:
                ids = results.boxes.id.int().tolist()
                for i, track_id in enumerate(ids):
                    x1, y1, x2, y2 = map(int, results.boxes.xyxy[i])
                    conf = float(results.boxes.conf[i])
                    last_tracks.append((x1, y1, x2, y2, conf, track_id))

        frame_count += 1

        # --- Logika target locking ---
        target_found_this_frame = False

        for (x1, y1, x2, y2, conf, track_id) in last_tracks:
            if locked_id is None:
                # belum ada target -> kunci orang pertama yang terdeteksi
                locked_id = track_id
                last_seen_time = time.time()

            is_target = (track_id == locked_id)

            if is_target:
                target_found_this_frame = True
                last_seen_time = time.time()

                centroid_x = (x1 + x2) // 2
                centroid_y = (y1 + y2) // 2

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.circle(frame, (centroid_x, centroid_y), 6, (0, 0, 255), -1)
                cv2.putText(frame, f'LOCKED ID:{track_id}', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                # --- Arah relatif terhadap tengah frame ---
                offset_x = centroid_x - frame_center_x
                if offset_x < -30:
                    direction = "KIRI"
                elif offset_x > 30:
                    direction = "KANAN"
                else:
                    direction = "LURUS"

                cv2.putText(frame, f'Arah: {direction}', (10, 75),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

                # TODO: kirim (direction, centroid_x, centroid_y) via serial ke ESP32 di sini

            else:
                # orang lain, bukan target -> gambar netral (abu-abu)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (100, 100, 100), 1)
                cv2.putText(frame, f'ID:{track_id}', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 100), 1)

        # --- Cek target hilang ---
        if locked_id is not None and not target_found_this_frame:
            elapsed = time.time() - last_seen_time
            if elapsed > LOST_TIMEOUT:
                cv2.putText(frame, 'TARGET HILANG', (10, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                # TODO: kirim perintah STOP ke ESP32 di sini
                locked_id = None
            else:
                cv2.putText(frame, f'Mencari... ({elapsed:.1f}s)', (10, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        cv2.putText(frame, f'FPS: {fps:.1f}', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        cv2.putText(frame, f'Orang terdeteksi: {len(last_tracks)}', (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

        ret, buffer = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')


@app.route('/video')
def video():
    return Response(generate_frames(),
                     mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/')
def index():
    return '<h1>VERY IRAWATI HAMA TEKNIK ELEKTRO UNESA</h1><img src="/video" width="320">'


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)