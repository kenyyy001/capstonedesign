import cv2
import os
from ultralytics import YOLO

# 1. Manajemen Model (Otomatis konversi ke format ONNX yang ringan untuk CPU)
pt_model_path = "yolov8n.pt"
onnx_model_path = "yolov8n.onnx"

# Jika file .onnx belum ada, buat otomatis dari model .pt
if not os.path.exists(onnx_model_path):
    print("Mengonversi model ke format ONNX untuk optimasi CPU...")
    model_init = YOLO(pt_model_path)
    model_init.export(format="onnx", imgsz=640) # Mengunci resolusi input di 640x640

# Muat model versi ONNX yang sudah dioptimalkan
model = YOLO(onnx_model_path, task="detect")

# 2. Konfigurasi Kamera OpenCV
cap = cv2.VideoCapture(0)
# Menggunakan resolusi HD standar (640x480) karena RAM 6GB Anda sangat mumpuni
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("Error: Kamera tidak dapat diakses.")
    exit()

print("Sistem Lock & Tracking Aktif... Tekan 'q' untuk keluar.")

while True:
    success, frame = cap.read()
    if not success:
        print("Gagal membaca frame kamera.")
        break

    # 3. Proses Tracking dengan Algoritma ByteTrack (Ringan & Cepat)
    # persist=True mengunci ID objek agar tidak tertukar saat bergerak
    results = model.track(frame, persist=True, tracker="bytetrack.yaml", verbose=False)

    # 4. Filter Objek yang Akan Dikunci (Locking)
    if results[0].boxes.id is not None:
        boxes = results[0].boxes.xyxy.int().cpu().tolist()
        class_ids = results[0].boxes.cls.int().cpu().tolist()
        track_ids = results[0].boxes.id.int().cpu().tolist()

        for box, class_id, track_id in zip(boxes, class_ids, track_ids):
            x1, y1, x2, y2 = box
            nama_objek = model.names[class_id]

            # TARGET LOCK: Silakan ganti 'person' dengan objek lain (misal: 'car', 'dog', 'bottle')
            if nama_objek == "person":
                # Hitung titik tengah objek (Center Point) untuk kebutuhan mekanik/robotik ke depan
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                
                label_lock = f"LOCKED ID:{track_id} | X:{center_x} Y:{center_y}"

                # Gambar kotak pengunci berwarna MERAH
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                # Gambar titik tengah target (Titik bidik)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                # Tampilkan text koordinat di atas kotak objek
                cv2.putText(frame, label_lock, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    else:
        # Jika target lock hilang atau sedang mencari
        cv2.putText(frame, "STATUS: SEARCHING TARGET...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # 5. Tampilkan Visualisasi Tracking
    cv2.imshow("Orange Pi 6GB - YOLO Tracking", frame)

    # Keluar jika menekan tombol 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
