
#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Twist


def clamp(value, low, high):
    return max(low, min(high, value))


class FollowControllerNode(Node):

    def __init__(self):
        super().__init__('follow_controller_node')

        # TRACKING
        self.declare_parameter('offset_alpha', 0.20)
        self.declare_parameter('yaw_deadband', 0.04)

        # ROTATION PD
        self.declare_parameter('kp_yaw', 0.65)
        self.declare_parameter('kd_yaw', 0.04)
        self.declare_parameter('max_yaw', 0.35)

        # ROTATION ACCELERATION LIMIT
        self.declare_parameter('yaw_accel', 0.7)

        # DISTANCE
        self.declare_parameter('target_distance', 0.60)
        self.declare_parameter('distance_deadband', 0.05)
        self.declare_parameter('kp_distance', 0.6)
        self.declare_parameter('max_forward', 0.20)
        self.declare_parameter('max_backward', 0.15)

        # LINEAR ACCELERATION LIMIT
        self.declare_parameter('linear_accel', 0.35)

        self.declare_parameter('lost_timeout_s', 0.35)

        self.alpha = self.get_parameter('offset_alpha').value
        self.yaw_deadband = self.get_parameter('yaw_deadband').value

        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.kd_yaw = self.get_parameter('kd_yaw').value
        self.max_yaw = self.get_parameter('max_yaw').value
        self.yaw_accel = self.get_parameter('yaw_accel').value

        self.target_distance = self.get_parameter('target_distance').value
        self.distance_deadband = self.get_parameter(
            'distance_deadband').value
        self.kp_distance = self.get_parameter('kp_distance').value
        self.max_forward = self.get_parameter('max_forward').value
        self.max_backward = self.get_parameter('max_backward').value

        self.linear_accel = self.get_parameter('linear_accel').value
        self.lost_timeout = self.get_parameter('lost_timeout_s').value

        self.target_locked = False
        self.position = None
        self.last_position_time = None

        self.filtered_offset = None
        self.previous_error = 0.0
        self.previous_time = None

        self.current_yaw = 0.0
        self.current_linear = 0.0

        self.create_subscription(
            Bool, '/vision/target_locked',
            self.locked_callback, 10)

        self.create_subscription(
            Point, '/vision/target_position',
            self.position_callback, 10)

        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            'Barcode Follow Controller - Smooth Rotation Mode Ready')

    def locked_callback(self, msg):
        self.target_locked = msg.data

    def position_callback(self, msg):
        self.position = msg
        self.last_position_time = self.get_clock().now()

    def position_is_fresh(self):
        if self.last_position_time is None:
            return False

        age = (
            self.get_clock().now() -
            self.last_position_time
        ).nanoseconds * 1e-9

        return age < self.lost_timeout

    def slew(self, current, target, accel, dt):
        maximum_change = accel * dt

        return current + clamp(
            target - current,
            -maximum_change,
            maximum_change
        )

    def compute_yaw(self, error, dt):

        # Tidak melakukan koreksi jika barcode sudah di tengah
        if abs(error) < self.yaw_deadband:
            self.previous_error = error
            return 0.0

        derivative = (error - self.previous_error) / max(dt, 0.001)
        self.previous_error = error

        output = (
            -self.kp_yaw * error
            -self.kd_yaw * derivative
        )

        return clamp(
            output,
            -self.max_yaw,
            self.max_yaw
        )

    def compute_distance(self, distance):

        error = distance - self.target_distance

        if abs(error) <= self.distance_deadband:
            return 0.0

        # Positif berarti maju secara konseptual
        speed = self.kp_distance * error

        return clamp(
            speed,
            -self.max_backward,
            self.max_forward
        )

    def control_loop(self):

        now = self.get_clock().now()

        if self.previous_time is None:
            dt = 0.05
        else:
            dt = (
                now - self.previous_time
            ).nanoseconds * 1e-9

        self.previous_time = now
        dt = clamp(dt, 0.01, 0.10)

        cmd = Twist()

        if not (
            self.target_locked
            and self.position is not None
            and self.position_is_fresh()
        ):
            # Target hilang: berhenti dengan aman
            self.current_yaw = 0.0
            self.current_linear = 0.0
            self.filtered_offset = None
            self.previous_error = 0.0

            self.cmd_pub.publish(cmd)
            return

        offset_raw = float(self.position.x)
        distance = float(self.position.y)

        if not (
            math.isfinite(offset_raw)
            and math.isfinite(distance)
            and 0.0 < distance <= 5.0
        ):
            self.current_yaw = 0.0
            self.current_linear = 0.0
            self.filtered_offset = None
            self.cmd_pub.publish(cmd)
            return

        # 1. Filter posisi barcode
        if self.filtered_offset is None:
            self.filtered_offset = offset_raw
        else:
            self.filtered_offset = (
                self.alpha * offset_raw
                + (1.0 - self.alpha) * self.filtered_offset
            )

        error = self.filtered_offset

        # 2. Kontrol rotasi
        target_yaw = self.compute_yaw(error, dt)

        self.current_yaw = self.slew(
            self.current_yaw,
            target_yaw,
            self.yaw_accel,
            dt
        )

        # 3. Kontrol jarak
        target_linear = self.compute_distance(distance)

        self.current_linear = self.slew(
            self.current_linear,
            target_linear,
            self.linear_accel,
            dt
        )

        # 4. Kirim perintah
        # Mapping sesuai ESP32 saat ini:
        # keyboard W = cmdY positif
        # cmdY = -linear.y
        # Maka maju dikirim sebagai linear.y negatif.
        cmd.linear.x = 0.0
        cmd.linear.y = -self.current_linear
        cmd.angular.z = self.current_yaw

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)
    node = FollowControllerNode()

    try:
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
