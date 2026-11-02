#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

import cv2
import numpy as np
import threading
import time

from flask import Flask, Response

flask_app = Flask(__name__)
frame_lock = threading.Lock()
output_frame = None  
active_clients = 0  # Hitung jumlah browser yang menonton

# --- THREAD KELAS UNTUK MEMBACA KAMERA TANPA BUFFER (REAL-TIME) ---
class RealTimeCamera:
    def __init__(self, source, w, h):
        self.cap = cv2.VideoCapture(source)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        self.ret = False
        self.frame = None
        self.running = True
        
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

    def _update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.ret = ret
                self.frame = frame  # Timpa frame lama secara brutal
            else:
                time.sleep(0.01)

    def read(self):
        return self.ret, self.frame

    def release(self):
        self.running = False
        self.cap.release()

def generate_mjpeg():
    global output_frame, active_clients
    active_clients += 1
    try:
        while True:
            with frame_lock:
                if output_frame is None:
                    time.sleep(0.03)
                    continue
                jpeg_bytes = output_frame 

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg_bytes + b'\r\n')
            time.sleep(0.04)  # Batasi visual browser di ~25 FPS
    finally:
        active_clients -= 1

@flask_app.route('/video')
def video_route():
    return Response(generate_mjpeg(), mimetype='multipart/x-mixed-replace; boundary=frame')

@flask_app.route('/')
def index_route():
    return '<html><body style="margin:0;background:black;"><img src="/video" style="width:100vw;height:100vh;object-fit:contain;"></body></html>'

def run_flask():
    flask_app.run(host='0.0.0.0', port=5000, threaded=True, use_reloader=False)


# --- NODE ROS2 ---
class ArucoLockNode(Node):
    def __init__(self):
        super().__init__('aruco_lock_node')

        self.declare_parameter('camera_source', '/dev/video0')
        self.declare_parameter('frame_width', 320)  
        self.declare_parameter('frame_height', 240) 
        self.declare_parameter('marker_size_m', 0.05)
        self.declare_parameter('target_marker_id', -1)
        self.declare_parameter('camera_focal_px', 290.0) 

        camera_source = self.get_parameter('camera_source').value
        self.frame_w = self.get_parameter('frame_width').value
        self.frame_h = self.get_parameter('frame_height').value
        self.marker_size_m = self.get_parameter('marker_size_m').value
        self.target_marker_id = self.get_parameter('target_marker_id').value
        self.focal_px = self.get_parameter('camera_focal_px').value

        # Panggil Kamera Real-time Terpisah
        self.cam = RealTimeCamera(camera_source, self.frame_w, self.frame_h)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.minMarkerPerimeterRate = 0.08  # Abaikan marker yang terlalu jauh/kecil
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.locked_pub = self.create_publisher(Bool, '/vision/target_locked', qos_profile)
        self.position_pub = self.create_publisher(Point, '/vision/target_position', qos_profile)

        self.locked_id = None
        
        # Jalankan pemrosesan secepat mungkin (tanpa timer kaku) untuk performa penuh
        self.create_timer(1.0 / 30.0, self.process_frame)
        self.get_logger().info('Aruco Lock Node REALTIME siap.')

    def process_frame(self):
        global output_frame, active_clients

        ret, frame = self.cam.read()
        if not ret or frame is None:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        target_found = False

        if ids is not None:
            ids_flat = ids.flatten()
            if self.locked_id is None:
                self.locked_id = self.target_marker_id if self.target_marker_id != -1 else int(ids_flat[0])

            if self.locked_id in ids_flat:
                idx = list(ids_flat).index(self.locked_id)
                marker_corners = corners[idx][0]
                target_found = True

                center_x = float(np.mean(marker_corners[:, 0]))
                center_y = float(np.mean(marker_corners[:, 1]))
                offset_x = (center_x - (self.frame_w / 2.0)) / (self.frame_w / 2.0)

                side_lengths = [np.linalg.norm(marker_corners[i] - marker_corners[(i + 1) % 4]) for i in range(4)]
                avg_side_px = float(np.mean(side_lengths))
                distance_m = (self.marker_size_m * self.focal_px) / avg_side_px if avg_side_px > 0 else 0.0

                pos_msg = Point(x=offset_x, y=distance_m, z=0.0)
                self.position_pub.publish(pos_msg)

                # Hanya gambar overlay jika ada browser aktif yang menonton (Hemat CPU!)
                if active_clients > 0:
                    cv2.polylines(frame, [marker_corners.astype(int)], True, (0, 0, 255), 2)
                    cv2.circle(frame, (int(center_x), int(center_y)), 4, (0, 0, 255), -1)

        self.locked_pub.publish(Bool(data=target_found))

        # Hanya kompres gambar jika ada browser yang melihat web
        if active_clients > 0:
            ret_enc, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
            if ret_enc:
                with frame_lock:
                    output_frame = buffer.tobytes()

    def destroy_node(self):
        self.cam.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoLockNode()
    threading.Thread(target=run_flask, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
