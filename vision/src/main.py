import cv2
from ultralytics import YOLO

model = YOLO("yolov8n.pt")
cap = cv2.VideoCapture(0)  # Gunakan webcam

locked_id = None  # Menyimpan ID objek yang dikunci
clicked_coords = None  # Menyimpan koordinat klik mouse


# Fungsi callback untuk menangkap klik mouse
def mouse_click(event, x, y, flags, param):
    global clicked_coords
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_coords = (x, y)


cv2.namedWindow("YOLOv8 Object Locking")
cv2.setMouseCallback("YOLOv8 Object Locking", mouse_click)

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break

    # Jalankan tracking bawaan YOLOv8
    results = model.track(frame, persist=True, tracker="bytetrack.yaml", verbose=False)

    if results[0].boxes.id is not None:
        boxes = results[0].boxes.xyxy.int().cpu().tolist()
        ids = results[0].boxes.id.int().cpu().tolist()

        # Logika 1: Menentukan ID mana yang mau dikunci berdasarkan klik mouse
        if clicked_coords:
            cx, cy = clicked_coords
            for box, obj_id in zip(boxes, ids):
                x1, y1, x2, y2 = box
                if x1 <= cx <= x2 and y1 <= cy <= y2:  # Jika klik berada di dalam box
                    locked_id = obj_id
                    print(f"Target Terkunci! ID: {locked_id}")
                    break
            clicked_coords = None  # Reset klik

        # Logika 2: Memantau objek yang sedang dikunci
        for box, obj_id in zip(boxes, ids):
            x1, y1, x2, y2 = box
            # Hitung titik tengah target
            center_x = int((x1 + x2) / 2)
            center_y = int((y1 + y2) / 2)

            if obj_id == locked_id:
                # Beri visualisasi khusus untuk objek yang dikunci (Warna Merah)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.putText(
                    frame,
                    f"LOCKED ID: {obj_id}",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2,
                )

                # DI SINI: Kirim data center_x dan center_y ke sistem Anda (misal Gimbal/Servo)
            else:
                # Objek lain yang tidak dikunci (Warna Hijau biasa)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)

    cv2.imshow("YOLOv8 Object Locking", frame)

    # Tekan 'r' untuk reset lock, atau 'q' untuk keluar
    key = cv2.waitKey(1) & 0xFF
    if key == ord("r"):
        locked_id = None
        print("Lock Direset.")
    elif key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
