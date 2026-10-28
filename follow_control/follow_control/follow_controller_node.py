#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Twist


class FollowControllerNode(Node):
    def __init__(self):
        super().__init__('follow_controller_node')

        # ---------------- Parameter (bisa diubah lewat launch / ros2 param) ----------------
        self.declare_parameter('desired_distance_m', 1.0)    # jarak yang ingin dijaga
        self.declare_parameter('distance_deadband_m', 0.15)  # selisih jarak yang dianggap "sudah pas"
        self.declare_parameter('kp_distance', 0.8)           # gain maju/mundur
        self.declare_parameter('kp_yaw', 1.2)                # gain putar
        self.declare_parameter('offset_deadband', 0.10)      # zona mati offset arah (0..1)
        self.declare_parameter('max_forward', 0.6)           # batas perintah maju (0..1)
        self.declare_parameter('max_yaw', 0.8)               # batas perintah putar (0..1)
        self.declare_parameter('min_command', 0.25)          # perintah minimum agar motor tidak stall
        self.declare_parameter('allow_reverse', False)       # False = kalau terlalu dekat robot cuma berhenti
        self.declare_parameter('lost_timeout_s', 0.5)        # data vision lebih tua dari ini = dianggap hilang
        self.declare_parameter('max_valid_distance_m', 5.0)  # jarak di atas ini dianggap noise

        self.desired_distance = self.get_parameter('desired_distance_m').value
        self.distance_deadband = self.get_parameter('distance_deadband_m').value
        self.kp_distance = self.get_parameter('kp_distance').value
        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.offset_deadband = self.get_parameter('offset_deadband').value
        self.max_forward = self.get_parameter('max_forward').value
        self.max_yaw = self.get_parameter('max_yaw').value
        self.min_command = self.get_parameter('min_command').value
        self.allow_reverse = self.get_parameter('allow_reverse').value
        self.lost_timeout = self.get_parameter('lost_timeout_s').value
        self.max_valid_distance = self.get_parameter('max_valid_distance_m').value

        # ---------------- State ----------------
        self.target_locked = False
        self.position = None
        self.last_position_time = None
        self.last_state = None

        # ---------------- Topic ----------------
        self.create_subscription(Bool, '/vision/target_locked', self.locked_callback, 10)
        self.create_subscription(Point, '/vision/target_position', self.position_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(1.0 / 20.0, self.control_loop)  # 20 Hz

        self.get_logger().info(
            f'Follow controller siap. Jarak target: {self.desired_distance:.2f} m')

    # ------------------------------------------------------------------
    def locked_callback(self, msg):
        self.target_locked = msg.data

    def position_callback(self, msg):
        self.position = msg
        self.last_position_time = self.get_clock().now()

    def position_is_fresh(self):
        if self.last_position_time is None:
            return False
        age = (self.get_clock().now() - self.last_position_time).nanoseconds * 1e-9
        return age < self.lost_timeout

    # ------------------------------------------------------------------
    def apply_min(self, value):
        """Kalau perintah tidak nol tapi terlalu kecil, naikkan ke minimum supaya motor tidak stall."""
        if value == 0.0:
            return 0.0
        if abs(value) < self.min_command:
            return math.copysign(self.min_command, value)
        return value

    def compute_forward(self, distance):
        error = distance - self.desired_distance
        if abs(error) <= self.distance_deadband:
            return 0.0
        value = self.kp_distance * error
        if not self.allow_reverse:
            value = max(0.0, value)
        value = max(-self.max_forward, min(self.max_forward, value))
        return self.apply_min(value)

    def compute_yaw(self, offset):
        if abs(offset) <= self.offset_deadband:
            return 0.0
        # marker di kanan (offset > 0) -> putar kanan = angular.z NEGATIF
        value = -self.kp_yaw * offset
        value = max(-self.max_yaw, min(self.max_yaw, value))
        return self.apply_min(value)

    # ------------------------------------------------------------------
    def control_loop(self):
        cmd = Twist()  # default: semua nol = STOP
        state = 'HILANG'

        if self.target_locked and self.position is not None and self.position_is_fresh():
            offset = self.position.x
            distance = self.position.y
            if 0.0 < distance <= self.max_valid_distance:
                cmd.linear.x = self.compute_forward(distance)
                cmd.angular.z = self.compute_yaw(offset)
                state = 'MENGIKUTI'

        self.cmd_pub.publish(cmd)

        if state != self.last_state:
            self.get_logger().info(f'Status: {state}')
            self.last_state = state


def main(args=None):
    rclpy.init(args=args)
    node = FollowControllerNode()
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
