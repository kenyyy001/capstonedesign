#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import threading
import time

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
                self.frame = frame
            else:
                time.sleep(0.01)

    def read(self):
        return self.ret, self.frame

    def release(self):
        self.running = False
        self.cap.release()

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

        self.cam = RealTimeCamera(camera_source, self.frame_w, self.frame_h)
        self.bridge = CvBridge()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.minMarkerPerimeterRate = 0.08
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.locked_pub = self.create_publisher(Bool, '/vision/target_locked', qos_profile)
        self.position_pub = self.create_publisher(Point, '/vision/target_position', qos_profile)
        # Tambahan publisher untuk mengirim gambar mentah ke Laptop via ROS2
        self.image_pub = self.create_publisher(Image, '/vision/camera_image', qos_profile)

        self.locked_id = None
        self.create_timer(1.0 / 30.0, self.process_frame)
        self.get_logger().info('Aruco Lock Node Ringan (Tanpa Flask) Siap.')

    def process_frame(self):
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

                # Gambar info tracking langsung pada frame mentah
                cv2.polylines(frame, [marker_corners.astype(int)], True, (0, 0, 255), 2)
                cv2.circle(frame, (int(center_x), int(center_y)), 4, (0, 0, 255), -1)
                cv2.putText(frame, f'ID:{self.locked_id} {distance_m:.2f}m', (int(center_x) - 40, int(center_y) - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        self.locked_pub.publish(Bool(data=target_found))

        # Kirim frame mentah berkecepatan tinggi ke Laptop melalui ROS2 topic
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(img_msg)
        except Exception as e:
            pass

    def destroy_node(self):
        self.cam.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoLockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
