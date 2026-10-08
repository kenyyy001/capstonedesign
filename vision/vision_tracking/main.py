"""
Generator Marker ArUco - jalankan SEKALI untuk membuat gambar marker,
lalu cetak dan tempelkan di kaki/pergelangan kaki target (pasien simulasi).

Cara pakai:
    python3 generate_marker.py

Hasilnya: file 'marker_id0.png' - cetak di kertas, ukur sisi marker setelah
dicetak (dalam meter), lalu isi angka itu ke parameter 'marker_size_m'
saat menjalankan aruco_lock_node.
"""
import cv2
import numpy as np

MARKER_ID = 0
MARKER_SIZE_PX = 400  # resolusi gambar, bukan ukuran fisik cetak

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
marker_image = np.zeros((MARKER_SIZE_PX, MARKER_SIZE_PX), dtype=np.uint8)
cv2.aruco.generateImageMarker(aruco_dict, MARKER_ID, MARKER_SIZE_PX, marker_image, 1)

cv2.imwrite(f'marker_id{MARKER_ID}.png', marker_image)
print(f"Marker ID {MARKER_ID} disimpan sebagai marker_id{MARKER_ID}.png")
print("Cetak file ini, UKUR panjang sisi marker setelah dicetak (dalam meter),")
print("lalu masukkan angka itu ke parameter marker_size_m saat menjalankan node.")