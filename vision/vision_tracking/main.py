#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
import cv2
import numpy as np
import threading
import time
import logging

from flask import Flask, Response


# ---------------- Flask app (jalan di thread terpisah) ----------------
flask_app = Flask(__name__)
frame_lock = threading.Lock()
output_frame = None
frame_seq = 0

stream_cond = threading.Condition()
latest_jpeg = None
latest_seq = 0

STREAM_FPS = 12
STREAM_W, STREAM_H = 480, 360
JPEG_QUALITY = 35

def encoder_loop():
    global latest_jpeg, latest_seq
    last_seq = -1
    period = 1.0 / STREAM_FPS
    while True:
        t0 = time.time()
        with frame_lock:
            frame = output_frame
            seq = frame_seq
        if frame is not None and seq != last_seq:
            last_seq = seq
            small = cv2.resize(frame, (STREAM_W, STREAM_H), interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])

            if ok:
                with stream_cond:
                    latest_jpeg = buf.tobytes()
                    latest_seq += 1
                    stream_cond.notify_all()
        dt = time.time() - t0
        if dt < period:
            time.sleep(period - dt)

def generate_mjpeg():
    last = -1
    while True:
        with stream_cond:
            stream_cond.wait_for(lambda: latest_seq != last, timeout=1.0)
            if latest_seq == last:
                continue
            last = latest_seq
            data = latest_jpeg

        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + data + b'\r\n')

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
    <body>
        <img id="feed" src="/snapshot">
        <script>
            const img = document.getElementById('feed');
            function refresh() {
                const next = new Image();
                next.onload = () => { img.src = next.src; refresh(); };
                next.onerror = () => { setTimeout(refresh, 200); };
                next.src = '/snapshot?t=' + Date.now();
            }
            refresh();
        </script>
    </body>
    </html>
    '''


def run_flask():
    flask_app.run(host='0.0.0.0', port=5000, threaded=True, use_reloader=False)

class LatestFrameCapture:
    def __init__(self, src, w, h, fps=30, fourcc='MJPG'):
        self.cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
        if fourcc:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        fcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        self.info = (f"{''.join(chr((fcc >> 8 * i) & 0xFF) for i in range(4))} "
                     f"{int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
                     f"{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} "
                     f"@{self.cap.get(cv2.CAP_PROP_FPS):.0f}")
        self.lock = threading.Lock()
        self.frame = None
        self.seq = 0
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def isOpened(self):
        return self.cap.isOpened()

    def _loop(self):
        while self.running:
            ok, f = self.cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            with self.lock:
                self.frame = f
                self.seq += 1

    def read(self, last_seq):
        with self.lock:
            if self.frame is None or self.seq == last_seq:
                return None, last_seq
            return self.frame, self.seq

    def release(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.cap.release()

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

        self.grabber = LatestFrameCapture(camera_source, self.frame_w, self.frame_h, 30)
        self.get_logger().info(f'Kamera aktif: {self.grabber.info}')
        if not self.grabber.isOpened():
            self.get_logger().error(f'Kamera {camera_source} tidak bisa dibuka!')
        self.last_seq = -1
        self._t_log = time.time()
        self._n = 0
        self._acc = 0.0

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.locked_pub = self.create_publisher(Bool, '/vision/target_locked', 10)
        self.position_pub = self.create_publisher(Point, '/vision/target_position', 10)

        self.locked_id = None

        timer_period = 1.0 / 30.0
        self.timer = self.create_timer(1.0 / 60.0, self.process_frame)

        self.get_logger().info('Aruco Lock Node siap. Buka http://<IP_INI>:5000 untuk visual.')

    def process_frame(self):
        global output_frame, frame_seq

        frame, seq = self.grabber.read(self.last_seq)
        if frame is None:
            return
        self.last_seq = seq

        t0 = time.perf_counter()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        self._acc += time.perf_counter() - t0
        self._n += 1
        now = time.time()
        if now - self._t_log >= 2.0:
            self.get_logger().info(
                f'FPS proses={self._n / (now - self._t_log):.1f} | deteksi={1000 * self._acc / self._n:.1f} ms')
            self._t_log, self._n, self._acc = now, 0, 0.0

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

                frame_center_x = frame.shape[1] / 2.0
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
            output_frame = frame
            frame_seq += 1

    def destroy_node(self):
        self.grabber.release()
        super().destroy_node()


def main(args=None):

    def run_flask():
        logging.getLogger('werkzeug').setLevel(logging.ERROR)
        flask_app.run(host='0.0.0.0', port=5000, threaded=True, use_reloader=False)

    rclpy.init(args=args)
    node = ArucoLockNode()

    threading.Thread(target=encoder_loop, daemon=True).start()

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
