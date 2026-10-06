import cv2

# 1. Inisialisasi Kamera (0 untuk webcam bawaan)
cap = cv2.VideoCapture(0)

# 2. Membuat Tracker (CSRT sangat bagus untuk mengunci satu objek dengan kuat)
# Catatan: Jika menggunakan OpenCV versi lama, gunakan cv2.TrackerCSRT_create()
tracker = cv2.TrackerCSRT_create()

# Ambil frame pertama untuk memilih objek yang ingin dikunci
success, frame = cap.read()
if not success:
    print("Gagal membuka kamera.")
    exit()

# 3. Pilih Objek (Klik & seret mouse untuk membuat kotak, lalu tekan ENTER atau SPACE)
print("Silakan pilih objek di jendela pop-up, lalu tekan ENTER.")
bbox = cv2.selectROI("Kunci Objek", frame, fromCenter=False, showCrosshair=True)

# Inisialisasi tracker dengan objek yang sudah dipilih
tracker.init(frame, bbox)
cv2.destroyWindow("Kunci Objek")

while True:
    success, frame = cap.read()
    if not success:
        break

    # 4. Update posisi objek yang dikunci pada frame baru
    timer = cv2.getTickCount()
    ret, bbox = tracker.update(frame)
    
    # Menghitung Frame Per Second (FPS)
    fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)

    # 5. Jika objek berhasil dilacak, gambar kotak pengunci
    if ret:
        # Koordinat kotak: x, y, lebar, tinggi
        p1 = (int(bbox[0]), int(bbox[1]))
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        
        # Gambar kotak hijau di sekeliling objek
        cv2.rectangle(frame, p1, p2, (0, 255, 0), 2, 1)
        cv2.putText(frame, "STATUS: LOCKED", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    else:
        # Jika objek hilang atau terhalang
        cv2.putText(frame, "STATUS: LOST", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Tampilkan info FPS pada layar
    cv2.putText(frame, f"FPS: {int(fps)}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    # Tampilkan hasil tracking ke layar
    cv2.imshow("Object Tracking & Locking", frame)

    # Tekan tombol 'q' untuk keluar dari program
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Lepaskan kamera dan tutup semua jendela jendela
cap.release()
cv2.destroyAllWindows()
