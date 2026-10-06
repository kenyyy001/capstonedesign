import cv2
import os
from flask import Flask, Response
from ultralytics import YOLO

app = Flask(__name__)

# 1. Muat model ONNX
onnx_model_path = "yolov8n.onnx"
model = YOLO(onnx_model_path, task="detect")

# 2. Konfigurasi Kamera (Resolusi dioptimalkan agar streaming lancar di Wi-Fi)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

def generate_frames():
    while True:
        success, frame = cap.read()
        if not success:
            break

        # 3. Proses Tracking YOLO (ByteTrack)
        results = model.track(frame, persist=True, tracker="bytetrack.yaml", verbose=False)

        # 4. Filter Objek & Gambarkan di Frame
        if results.boxes.id is not None:
            boxes = results.boxes.xyxy.int().cpu().tolist()
            class_ids = results.boxes.cls.int().cpu().tolist()
            track_ids = results.boxes.id.int().cpu().tolist()

            for box, class_id, track_id in zip(boxes, class_ids, track_ids):
                x1, y1, x2, y2 = box
                nama_objek = model.names[class_id]

                # Kunci objek 'person' (bisa Anda ganti sesuai kebutuhan robot)
                if nama_objek == "person":
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    
                    label_lock = f"LOCKED ID:{track_id} | X:{center_x} Y:{center_y}"
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                    cv2.putText(frame, label_lock, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    
                    # Cetak koordinat di terminal SSH Anda untuk debugging mekanik robot
                    print(f"[LOCKED] X: {center_x}, Y: {center_y}")

        # 5. Kompres frame kamera menjadi format JPEG untuk dikirim ke Web Browser
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        frame_bytes = buffer.tobytes()

        # Gabungkan frame menjadi sebuah chunk video stream
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/')
def index():
    # Mengirimkan response stream video ke browser
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    print("\n=======================================================")
    print(" Jendela GUI dinonaktifkan. Beralih ke MODE WEB STREAM.")
    print("=======================================================")
    # Menjalankan server Flask agar bisa diakses oleh perangkat satu jaringan WiFi
    # Port 5000 adalah port default Flask
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
