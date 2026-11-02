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

        # ---------------- YAW (orientasi badan) ----------------
        self.declare_parameter('kp_yaw', 1.0)
        self.declare_parameter('offset_enter', 0.18)
        self.declare_parameter('offset_exit', 0.08)
        self.declare_parameter('offset_smoothing_alpha', 0.3)
        self.declare_parameter('max_yaw', 0.6)
        self.declare_parameter('min_yaw_command', 0.22)

        # ---------------- STRAFE (gerak menyilang, HOLONOMIC) ----------------
        self.declare_parameter('kp_strafe', 1.0)
        self.declare_parameter('strafe_deadband', 0.05)   # mati dekat tengah
        self.declare_parameter('max_strafe', 0.30)
        self.declare_parameter('min_strafe_command', 0.10)

        # ---------------- JARAK (3 zona) ----------------
        self.declare_parameter('far_threshold_m', 0.3)
        self.declare_parameter('near_threshold_m', 0.2)
        self.declare_parameter('forward_speed', 0.35)
        self.declare_parameter('backward_speed', 0.30)
        self.declare_parameter('max_valid_distance_m', 5.0)

        self.declare_parameter('lost_timeout_s', 0.5)

        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.offset_enter = self.get_parameter('offset_enter').value
        self.offset_exit = self.get_parameter('offset_exit').value
        self.smoothing_alpha = self.get_parameter('offset_smoothing_alpha').value
        self.max_yaw = self.get_parameter('max_yaw').value
        self.min_yaw_command = self.get_parameter('min_yaw_command').value

        self.kp_strafe = self.get_parameter('kp_strafe').value
        self.strafe_deadband = self.get_parameter('strafe_deadband').value
        self.max_strafe = self.get_parameter('max_strafe').value
        self.min_strafe_command = self.get_parameter('min_strafe_command').value

        self.far_threshold = self.get_parameter('far_threshold_m').value
        self.near_threshold = self.get_parameter('near_threshold_m').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.backward_speed = self.get_parameter('backward_speed').value
        self.max_valid_distance = self.get_parameter('max_valid_distance_m').value

        self.lost_timeout = self.get_parameter('lost_timeout_s').value

        self.target_locked = False
        self.position = None
        self.last_position_time = None
        self.last_state = None

        self.filtered_offset = 0.0
        self.is_turning = False
        self.distance_state = 'STOP'

        self.create_subscription(Bool, '/vision/target_locked', self.locked_callback, 10)
        self.create_subscription(Point, '/vision/target_position', self.position_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(1.0 / 20.0, self.control_loop)

        self.get_logger().info(
            f'Follow controller v3 (holonomic strafe) siap. Zona jarak: maju>{self.far_threshold}m, '
            f'mundur<{self.near_threshold}m')

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

    def compute_yaw(self, filtered_offset):
        """Putar badan supaya lurus. Berjalan BERSAMAAN dengan strafe (lihat control_loop),
        bukan menunggu strafe selesai -- itu prinsip holonomic yang Anda minta."""
        magnitude = abs(filtered_offset)

        if self.is_turning:
            if magnitude < self.offset_exit:
                self.is_turning = False
        else:
            if magnitude > self.offset_enter:
                self.is_turning = True

        if not self.is_turning:
            return 0.0

        value = -self.kp_yaw * filtered_offset
        value = max(-self.max_yaw, min(self.max_yaw, value))

        if abs(value) < self.min_yaw_command:
            value = math.copysign(self.min_yaw_command, value)

        return value

    def compute_strafe(self, filtered_offset):
        """Gerak menyilang sebanding offset marker dari tengah frame.
        Dihitung dari offset yang SAMA dengan compute_yaw, di tick yang sama,
        sehingga robot langsung memotong miring sambil badan pelan-pelan lurus."""
        if abs(filtered_offset) <= self.strafe_deadband:
            return 0.0

        e = filtered_offset - math.copysign(self.strafe_deadband, filtered_offset)
        value = self.kp_strafe * e
        value = max(-self.max_strafe, min(self.max_strafe, value))

        if abs(value) < self.min_strafe_command:
            value = math.copysign(self.min_strafe_command, value)
            
        return value

    def compute_forward(self, distance):
        if distance > self.far_threshold:
            self.distance_state = 'FORWARD'
        elif distance < self.near_threshold:
            self.distance_state = 'BACKWARD'
        else:
            if self.distance_state != 'BACKWARD':
                self.distance_state = 'STOP'

        if self.distance_state == 'FORWARD':
            return self.forward_speed
        elif self.distance_state == 'BACKWARD':
            return -self.backward_speed
        return 0.0

    def control_loop(self):
        cmd = Twist()
        state = 'HILANG'

        if self.target_locked and self.position is not None and self.position_is_fresh():
            offset_raw = self.position.x
            distance = self.position.y

            a = self.smoothing_alpha
            self.filtered_offset = a * offset_raw + (1.0 - a) * self.filtered_offset

            if 0.0 < distance <= self.max_valid_distance:
                cmd.linear.x = self.compute_forward(distance)
                cmd.linear.y = self.compute_strafe(self.filtered_offset)
                cmd.angular.z = self.compute_yaw(self.filtered_offset)
                state = (f'MENGIKUTI (jarak_state={self.distance_state}, '
                         f'strafe={cmd.linear.y:.2f}, yaw={cmd.angular.z:.2f})')
        else:
            self.distance_state = 'STOP'
            self.is_turning = False
            self.filtered_offset = 0.0

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