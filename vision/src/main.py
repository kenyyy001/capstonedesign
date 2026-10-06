import cv2
import time

# 1. Inisialisasi Kamera
# Gunakan resolusi rendah (misal: 320x240 atau 640x480) agar beban CPU Orange Pi ringan
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 426)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# 2. Membuat Tracker menggunakan sintaks OpenCV Terbaru
# TrackerKCF sangat direkomendasikan untuk spesifikasi Orange Pi Zero 3W
tracker = cv2.TrackerKCF.create()

# Ambil frame awal untuk seleksi objek
success, frame = cap.read()
if not success:
    print("Error: Kamera tidak terdeteksi.")
    cap.release()
    exit()

# 3. GUI Seleksi Objek awal
print("=== PETUNJUK ===")
print("1. Drag mouse pada objek yang ingin dikunci.")
print("2. Tekan ENTER atau SPASI untuk konfirmasi lock.")
bbox = cv2.selectROI("Lock Objek", frame, fromCenter=False, showCrosshair=True)

# Inisialisasi Tracker dengan koordinat objek terpilih
tracker.init(frame, bbox)
cv2.destroyWindow("Lock Objek")

# Variabel untuk menghitung FPS secara akurat
prev_time = 0

while True:
    success, frame = cap.read()
    if not success:
        print("Gagal mengambil gambar dari kamera.")
        break

    # 4. Update Posisi Tracker
    # Mengembalikan status boolean (True/False) dan tuple koordinat baru
    is_tracked, bbox = tracker.update(frame)
    
    # 5. Visualisasi Hasil Penguncian Objek
    if is_tracked:
        # Unpack koordinat kotak pembatas (Bounding Box)
        x, y, w, h = [int(v) for v in bbox]
        
        # Gambar kotak target pengunci (Warna hijau)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, "STATUS: LOCKED", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    else:
        # Jika objek bergerak terlalu cepat atau terhalang (Warna merah)
        cv2.putText(frame, "STATUS: TARGET LOST", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # Menghitung & Menampilkan FPS aktual di Orange Pi
    current_time = time.time()
    fps = 1 / (current_time - prev_time) if (current_time - prev_time) > 0 else 0
    prev_time = current_time
    cv2.putText(frame, f"FPS: {int(fps)}", (15, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    # Tampilkan jendela tracking
    cv2.imshow("OrangePi 3W - Object Tracking", frame)

    # Tekan 'q' untuk keluar
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
