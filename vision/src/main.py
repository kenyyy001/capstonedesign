from flask import Flask, Response
from ultralytics import YOLO
import cv2
import time

app = Flask(__name__)

print("Loading model YOLOv8n...")
model = YOLO('yolov8n.pt')
print("Model siap.")

cap = cv2.VideoCapture(0)

# --- OPTIMASI 1: set resolusi capture langsung dari kamera (bukan resize belakangan) ---
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

if not cap.isOpened():
    print("ERROR: Kamera tidak bisa dibuka. Cek koneksi USB webcam.")

# --- OPTIMASI 3: skip beberapa frame, jangan deteksi di setiap frame ---
FRAME_SKIP = 2  # jalankan YOLO tiap 3 frame (0,3,6,...), sisanya pakai hasil terakhir
frame_count = 0
last_boxes = []  # simpan hasil deteksi terakhir untuk ditampilkan di frame yang di-skip

# untuk hitung FPS real
prev_time = time.time()
fps = 0.0


def generate_frames():
    global frame_count, last_boxes, prev_time, fps

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Gagal membaca frame dari kamera.")
            break

        # --- Hitung FPS aktual ---
        current_time = time.time()
        fps = 1.0 / (current_time - prev_time) if current_time != prev_time else fps
        prev_time = current_time

        # --- OPTIMASI 3: hanya jalankan YOLO tiap FRAME_SKIP+1 frame ---
        if frame_count % (FRAME_SKIP + 1) == 0:
            # --- OPTIMASI 2: imgsz lebih kecil untuk inferensi lebih cepat ---
            results = model(frame, verbose=False, imgsz=320)[0]

            last_boxes = []
            for box in results.boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                if cls_id == 0 and conf > 0.5:  # class 0 = person
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    last_boxes.append((x1, y1, x2, y2, conf))

        frame_count += 1

        # Gambar hasil deteksi terakhir (baik dari frame ini atau frame sebelumnya)
        for (x1, y1, x2, y2, conf) in last_boxes:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f'Person {conf:.2f}', (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.putText(frame, f'Terdeteksi: {len(last_boxes)} orang', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(frame, f'FPS: {fps:.1f}', (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

        ret, buffer = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')


@app.route('/video')
def video():
    return Response(generate_frames(),
                     mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/')
def index():
    return '<h1> Very irawati: Deteksi Person YOLOv8n (Optimized)</h1><img src="/video" width="320">'


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)