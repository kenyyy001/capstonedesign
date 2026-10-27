#!/usr/bin/env python3
"""
Serial Bridge Node
==================
Jembatan antara ROS2 dan firmware ESP32.

Subscribe:
  /cmd_vel (geometry_msgs/Twist), nilai ternormalisasi -1.0 .. 1.0
        linear.x  = maju (+)
        linear.y  = geser kiri (+)
        angular.z = putar kiri (+)

Kirim ke ESP32 lewat USB serial, 20x per detik (juga berfungsi sebagai heartbeat):
  V <vx> <vy> <w> <s>\\n
        vx, vy, w = input inverse kinematics firmware (linear_x, linear_y, angular_z)
                    firmware: linear_y = MAJU, linear_x = geser, angular_z = putar kiri (+)
        s         = skala kecepatan 0.0 .. 1.0

Keselamatan: kalau /cmd_vel berhenti datang lebih dari cmd_timeout_s,
bridge mengirim perintah NOL. Firmware juga punya watchdog sendiri.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist

import serial


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('linear_scale', 0.5)    # satuan firmware untuk translasi penuh (sama dengan tombol W = 0.5)
        self.declare_parameter('angular_scale', 9.0)   # satuan firmware untuk putar penuh (perlu tuning)
        self.declare_parameter('lateral_sign', -1.0)   # ROS y = kiri, firmware x = kanan (BELUM diverifikasi fisik)
        self.declare_parameter('yaw_sign', 1.0)        # ubah ke -1.0 kalau arah putar terbalik
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('send_rate_hz', 20.0)

        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baudrate').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.lateral_sign = self.get_parameter('lateral_sign').value
        self.yaw_sign = self.get_parameter('yaw_sign').value
        self.cmd_timeout = self.get_parameter('cmd_timeout_s').value
        rate = self.get_parameter('send_rate_hz').value

        self.ser = None
        self.ready_at = 0.0     # ESP32 reset saat port dibuka, tunggu sebentar sebelum kirim
        self.next_retry = 0.0

        self.last_lx = 0.0
        self.last_ly = 0.0
        self.last_az = 0.0
        self.last_cmd_time = 0.0  # 0 = belum pernah terima perintah -> kirim STOP

        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_timer(1.0 / rate, self.send_loop)

        self.get_logger().info(f'Serial bridge siap. Port: {self.port} @ {self.baud}')

    # ------------------------------------------------------------------
    def cmd_callback(self, msg):
        self.last_lx = msg.linear.x
        self.last_ly = msg.linear.y
        self.last_az = msg.angular.z
        self.last_cmd_time = time.time()

    def ensure_serial(self):
        if self.ser is not None:
            return
        if time.time() < self.next_retry:
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            self.ready_at = time.time() + 2.0
            self.get_logger().info(f'Port {self.port} terbuka, menunggu ESP32 siap...')
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'Gagal membuka {self.port}: {exc} (coba lagi 2 detik)')
            self.next_retry = time.time() + 2.0

    def close_serial(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
        self.ser = None
        self.next_retry = time.time() + 2.0

    # ------------------------------------------------------------------
    def send_loop(self):
        self.ensure_serial()
        if self.ser is None or time.time() < self.ready_at:
            return

        if time.time() - self.last_cmd_time > self.cmd_timeout:
            lx = ly = az = 0.0
        else:
            lx, ly, az = self.last_lx, self.last_ly, self.last_az

        vy = lx * self.linear_scale                          # maju -> linear_y firmware
        vx = self.lateral_sign * ly * self.linear_scale      # geser -> linear_x firmware
        w = self.yaw_sign * az * self.angular_scale          # putar -> angular_z firmware
        s = min(1.0, max(abs(lx), abs(ly), abs(az)))         # skala kecepatan

        line = f'V {vx:.4f} {vy:.4f} {w:.4f} {s:.3f}\n'
        try:
            self.ser.reset_input_buffer()  # buang teks monitor dari ESP32
            self.ser.write(line.encode('ascii'))
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'Gagal menulis ke serial: {exc}')
            self.close_serial()

    # ------------------------------------------------------------------
    def destroy_node(self):
        if self.ser is not None:
            try:
                for _ in range(3):
                    self.ser.write(b'V 0 0 0 0\n')
                    time.sleep(0.02)
            except (serial.SerialException, OSError):
                pass
            self.close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
