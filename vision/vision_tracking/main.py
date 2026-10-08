#!/usr/bin/env python3
"""
ArUco Lock Node - Human Leg Tracking via Marker (dengan visual streaming)
===========================================================================
Mengunci target manusia menggunakan marker ArUco yang ditempel di kaki/
pergelangan kaki pasien. Tambahan: visual kamera bisa dilihat dari browser
via Flask streaming (karena node ini headless, tidak ada cv2.imshow()).

Publish ROS2:
- /vision/target_locked   (std_msgs/Bool)
- /vision/target_position (geometry_msgs/Point)  x=offset, y=jarak(meter)

Visual:
- Buka http://<IP_ORANGEPI>:5000 dari browser laptop untuk lihat kamera.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

import cv2
import numpy as np
import threading
import time

from flask import Flask, Response


# ---------------- Flask app (jalan di thread terpisah) ----------------
flask_app = Flask(__name__)
frame_lock = threading.Lock()
output_frame = None  # frame terbaru (sudah digambar overlay), dibaca Flask


def generate_mjpeg():
    global output_frame
    while True:
        with frame_lock:
            if output_frame is None:
                time.sleep(0.05)
                continue
            frame_copy = output_frame.copy()

        ret, buffer = cv2.imencode('.jpg', frame_copy)
        if not ret:
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.03)


@flask_app.route('/video')
def video_route():
    return Response(generate_mjpeg(),
                     mimetype='multipart/x-mixed-replace; boundary=frame')


@flask_app.route('/')
def index_route():
    return '''
    <!DOCTYPE html>
    <html>
    <head>
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <title>ArUco Lock - Visual</title>
        <style>
            body { margin: 0; padding: 0; background: black; }
            img { display: block; width: 100vw; height: 100vh; object-fit: contain; }
        </style>
    </head>
    <body><img src="/video"></body>
    </html>
    '''


def run_flask():
    flask_app.run(host='0.0.0.0', port=5000, threaded=True, use_reloader=False)


# ---------------- Node ROS2 ----------------
class ArucoLockNode(Node):
    def __init__(self):
        super().__init__('aruco_lock_node')

        self.declare_parameter('camera_source', '/dev/video0')
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('marker_size_m', 0.05)
        self.declare_parameter('target_marker_id', -1)
        self.declare_parameter('camera_focal_px', 600.0)

        camera_source = self.get_parameter('camera_source').value
        self.frame_w = self.get_parameter('frame_width').value
        self.frame_h = self.get_parameter('frame_height').value
        self.marker_size_m = self.get_parameter('marker_size_m').value
        self.target_marker_id = self.get_parameter('target_marker_id').value
        self.focal_px = self.get_parameter('camera_focal_px').value

        self.cap = cv2.VideoCapture(camera_source)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_h)

        if not self.cap.isOpened():
            self.get_logger().error(f'Kamera {camera_source} tidak bisa dibuka!')

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.locked_pub = self.create_publisher(Bool, '/vision/target_locked', 10)
        self.position_pub = self.create_publisher(Point, '/vision/target_position', 10)

        self.locked_id = None

        timer_period = 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.process_frame)

        self.get_logger().info('Aruco Lock Node siap. Buka http://<IP_INI>:5000 untuk visual.')

    def process_frame(self):
        global output_frame

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Gagal membaca frame dari kamera.')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        target_found = False

        if ids is not None:
            ids_flat = ids.flatten()

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            if self.locked_id is None:
                if self.target_marker_id != -1:
                    if self.target_marker_id in ids_flat:
                        self.locked_id = self.target_marker_id
                        self.get_logger().info(f'Target dikunci: marker ID {self.locked_id}')
                else:
                    self.locked_id = int(ids_flat[0])
                    self.get_logger().info(f'Target dikunci: marker ID {self.locked_id}')

            if self.locked_id is not None and self.locked_id in ids_flat:
                idx = list(ids_flat).index(self.locked_id)
                marker_corners = corners[idx][0]

                target_found = True

                center_x = float(np.mean(marker_corners[:, 0]))
                center_y = float(np.mean(marker_corners[:, 1]))

                frame_center_x = self.frame_w / 2.0
                offset_x = (center_x - frame_center_x) / frame_center_x

                side_lengths = [
                    np.linalg.norm(marker_corners[i] - marker_corners[(i + 1) % 4])
                    for i in range(4)
                ]
                avg_side_px = float(np.mean(side_lengths))
                distance_m = (self.marker_size_m * self.focal_px) / avg_side_px if avg_side_px > 0 else 0.0

                pts = marker_corners.astype(int)
                cv2.polylines(frame, [pts], True, (0, 0, 255), 3)
                cv2.circle(frame, (int(center_x), int(center_y)), 6, (0, 0, 255), -1)
                cv2.putText(frame, f'LOCKED ID:{self.locked_id}', (int(center_x) - 60, int(center_y) - 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
                cv2.putText(frame, f'Jarak: {distance_m:.2f}m', (int(center_x) - 60, int(center_y) + 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                direction = "KIRI" if offset_x < -0.15 else "KANAN" if offset_x > 0.15 else "LURUS"
                cv2.putText(frame, f'Arah: {direction} (offset:{offset_x:.2f})', (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

                pos_msg = Point()
                pos_msg.x = offset_x
                pos_msg.y = distance_m
                pos_msg.z = 0.0
                self.position_pub.publish(pos_msg)

        status_text = f'Status: {"LOCKED" if target_found else "MENCARI..."}'
        status_color = (0, 255, 0) if target_found else (0, 165, 255)
        cv2.putText(frame, status_text, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        if self.locked_id is not None:
            cv2.putText(frame, f'Target ID: {self.locked_id}', (10, 95),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        locked_msg = Bool()
        locked_msg.data = target_found
        self.locked_pub.publish(locked_msg)

        with frame_lock:
            output_frame = frame.copy()

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLockNode()

    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
