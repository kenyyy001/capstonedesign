from flask import Flask, Response
from ultralytics import YOLO
import cv2
import threading
import time

app = Flask(__name__)

print("Loading model YOLOv8n...")
model = YOLO('yolov8n.pt')
print("Model siap.")

# Ganti sesuai device kamera yang benar (cek: ls /dev/video*)
CAMERA_SOURCE = '/dev/video2'

FRAME_SKIP = 1
MATCH_THRESHOLD = 0.5
HIST_UPDATE_RATE = 0.1

# --- Shared state antara thread kamera dan Flask (dilindungi lock) ---
lock = threading.Lock()
output_frame = None       # frame terbaru yang sudah digambar, siap ditampilkan
camera_running = True     # flag untuk stop thread dengan rapi


def compute_histogram(frame, box):
    x1, y1, x2, y2 = box
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(frame.shape[1], x2), min(frame.shape[0], y2)
    if x2 <= x1 or y2 <= y1:
        return None
    crop = frame[y1:y2, x1:x2]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [50, 60], [0, 180, 0, 256])
    cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
    return hist


def compare_histogram(hist1, hist2):
    if hist1 is None or hist2 is None:
        return 0.0
    return max(0.0, cv2.compareHist(hist1, hist2, cv2.HISTCMP_CORREL))


def camera_worker():
    """Thread TUNGGAL yang membaca kamera + jalankan YOLO. Tidak ada thread lain yang boleh sentuh cap."""
    global output_frame, camera_running

    cap = cv2.VideoCapture(CAMERA_SOURCE)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    if not cap.isOpened():
        print(f"ERROR: Kamera {CAMERA_SOURCE} tidak bisa dibuka.")
        return

    frame_count = 0
    last_tracks = []
    prev_time = time.time()
    fps = 0.0
    target_histogram = None

    consecutive_fail = 0

    while camera_running:
        ret, frame = cap.read()

        if not ret:
            consecutive_fail += 1
            print(f"Gagal membaca frame ({consecutive_fail}x), mencoba lagi...")
            # Coba buka ulang kamera kalau gagal berkali-kali (bukan langsung menyerah)
            if consecutive_fail >= 10:
                print("Membuka ulang koneksi kamera...")
                cap.release()
                time.sleep(1)
                cap = cv2.VideoCapture(CAMERA_SOURCE)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
                consecutive_fail = 0
            time.sleep(0.1)
            continue

        consecutive_fail = 0

        frame_h, frame_w = frame.shape[:2]
        frame_center_x = frame_w // 2

        current_time = time.time()
        fps = 1.0 / (current_time - prev_time) if current_time != prev_time else fps
        prev_time = current_time

        if frame_count % (FRAME_SKIP + 1) == 0:
            results = model.track(frame, persist=True, verbose=False,
                                   imgsz=320, conf=0.5, classes=[0])[0]
            last_tracks = []
            if results.boxes is not None and results.boxes.id is not None:
                ids = results.boxes.id.int().tolist()
                for i, track_id in enumerate(ids):
                    x1, y1, x2, y2 = map(int, results.boxes.xyxy[i])
                    conf = float(results.boxes.conf[i])
                    last_tracks.append((x1, y1, x2, y2, conf, track_id))
        frame_count += 1

        best_match_score = 0.0
        best_match_box = None

        if target_histogram is None and len(last_tracks) > 0:
            x1, y1, x2, y2, conf, track_id = last_tracks[0]
            target_histogram = compute_histogram(frame, (x1, y1, x2, y2))
        elif target_histogram is not None:
            for (x1, y1, x2, y2, conf, track_id) in last_tracks:
                hist = compute_histogram(frame, (x1, y1, x2, y2))
                score = compare_histogram(hist, target_histogram)
                if score > best_match_score:
                    best_match_score = score
                    best_match_box = (x1, y1, x2, y2, track_id, hist)

        for (x1, y1, x2, y2, conf, track_id) in last_tracks:
            is_target = (best_match_box is not None and best_match_box[3] == track_id
                         and best_match_score >= MATCH_THRESHOLD)
            if is_target:
                centroid_x, centroid_y = (x1 + x2) // 2, (y1 + y2) // 2
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.circle(frame, (centroid_x, centroid_y), 6, (0, 0, 255), -1)
                cv2.putText(frame, f'LOCKED (mirip:{best_match_score:.2f})', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                offset_x = centroid_x - frame_center_x
                direction = "KIRI" if offset_x < -30 else "KANAN" if offset_x > 30 else "LURUS"
                cv2.putText(frame, f'Arah: {direction}', (10, 75),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
                new_hist = best_match_box[5]
                if new_hist is not None:
                    target_histogram = cv2.addWeighted(
                        target_histogram, 1 - HIST_UPDATE_RATE, new_hist, HIST_UPDATE_RATE, 0)
                # TODO: kirim (direction, centroid_x, centroid_y) via serial ke ESP32 di sini
            else:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (100, 100, 100), 1)
                cv2.putText(frame, f'ID:{track_id}', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 100), 1)

        if target_histogram is not None and best_match_score < MATCH_THRESHOLD:
            cv2.putText(frame, f'Mencari target (skor:{best_match_score:.2f})...',
                        (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)

        cv2.putText(frame, f'FPS: {fps:.1f}', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        cv2.putText(frame, f'Orang terdeteksi: {len(last_tracks)}', (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

        # --- Simpan hasil ke variabel global dengan lock, supaya aman diakses banyak request ---
        with lock:
            output_frame = frame.copy()

    cap.release()
    print("Camera worker berhenti.")


def generate_frames():
    """Flask HANYA membaca output_frame yang sudah disiapkan camera_worker. Tidak sentuh cap sama sekali."""
    global output_frame
    while True:
        with lock:
            if output_frame is None:
                continue
            frame_copy = output_frame.copy()

        ret, buffer = cv2.imencode('.jpg', frame_copy)
        if not ret:
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

        time.sleep(0.03)  # batasi kecepatan pengiriman ke browser (~30fps max), tidak membebani sistem


@app.route('/video')
def video():
    return Response(generate_frames(),
                     mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/')
def index():
    return '''
    <!DOCTYPE html>
    <html>
    <head>
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <title>Person Tracking + Color Re-ID Lock</title>
        <style>
            body { margin: 0; padding: 0; background: black; }
            img { display: block; width: 100vw; height: 100vh; object-fit: contain; }
        </style>
    </head>
    <body><img src="/video"></body>
    </html>
    '''


if __name__ == '__main__':
    # Jalankan thread kamera SATU KALI saat program mulai, terlepas dari berapa banyak request web nanti
    camera_thread = threading.Thread(target=camera_worker, daemon=True)
    camera_thread.start()

    try:
        app.run(host='0.0.0.0', port=5000, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        camera_running = False
        camera_thread.join(timeout=3)