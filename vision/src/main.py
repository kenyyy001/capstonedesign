"""
TAHAP AWAL - Validasi Kamera + YOLO
=====================================
Tujuan: memastikan webcam terbaca dan model YOLO bisa mendeteksi
manusia secara real-time, sebelum lanjut ke tracking & target locking.

Cara pakai:
1. Jalankan: python3 tahap_awal_deteksi.py
2. Buka browser di LAPTOP (bukan di Raspi): http://<IP_RASPI>:5000
"""

from flask import Flask, Response
from ultralytics import YOLO
import cv2

app = Flask(__name__)

# Model otomatis ke-download saat pertama kali dijalankan (~6MB)
print("Loading model YOLOv8n...")
model = YOLO('yolov8n.pt')
print("Model siap.")

# index 0 -> /dev/video0 (webcam Logitech kamu)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("ERROR: Kamera tidak bisa dibuka. Cek koneksi USB webcam.")


def generate_frames():
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Gagal membaca frame dari kamera.")
            break

        frame = cv2.resize(frame, (640, 480))

        # jalankan deteksi YOLO di frame ini
        results = model(frame, verbose=False)[0]

        jumlah_orang = 0

        for box in results.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])

            # class id 0 di COCO dataset = "person"
            if cls_id == 0 and conf > 0.5:
                jumlah_orang += 1
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f'Person {conf:.2f}', (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # info jumlah orang terdeteksi di pojok kiri atas
        cv2.putText(frame, f'Terdeteksi: {jumlah_orang} orang', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        ret, buffer = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')


@app.route('/video')
def video():
    return Response(generate_frames(),
                     mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/')
def index():
    return '<h1>Tahap Awal: Deteksi Person YOLOv8n</h1><img src="/video" width="640">'


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)