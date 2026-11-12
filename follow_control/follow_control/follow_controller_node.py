import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Twist


class FollowControllerNode(Node):
    def __init__(self):
        super().__init__('follow_controller_node')

        self.declare_parameter('kp_yaw', 1.0)
        self.declare_parameter('offset_enter', 0.18)
        self.declare_parameter('offset_exit', 0.08)
        self.declare_parameter('offset_smoothing_alpha', 0.3)
        self.declare_parameter('max_yaw', 0.6)
        self.declare_parameter('min_yaw_command', 0.22)
        self.declare_parameter('offset_bias', 0.0)
        self.declare_parameter('yaw_deadband', 0.10)

        self.declare_parameter('kp_strafe', 0.5)
        self.declare_parameter('strafe_deadband', 0.08)   # mati dekat tengah
        self.declare_parameter('max_strafe', 0.20)
        self.declare_parameter('min_strafe_command', 0.0)

        self.declare_parameter('far_threshold_m', 0.3)
        self.declare_parameter('near_threshold_m', 0.2)
        self.declare_parameter('forward_speed', 0.35)
        self.declare_parameter('backward_speed', 0.30)
        self.declare_parameter('max_valid_distance_m', 5.0)
        self.declare_parameter('lost_timeout_s', 0.5)

        self.declare_parameter('lost_hold_s', 2.0)     # lama meneruskan perintah terakhir
        self.declare_parameter('lost_taper_s', 1.0)    # bagian akhir hold yang diturunkan linear ke 0
        self.declare_parameter('enable_yaw', False)    # False = hanya strafe, tanpa rotasi

        self.lost_hold = self.get_parameter('lost_hold_s').value
        self.lost_taper = self.get_parameter('lost_taper_s').value
        self.enable_yaw = self.get_parameter('enable_yaw').value

        self.last_cmd = (0.0, 0.0, 0.0)
        self.last_seen = None
        self.filter_ready = False

        self.kp_yaw = self.get_parameter('kp_yaw').value
        self.offset_enter = self.get_parameter('offset_enter').value
        self.offset_exit = self.get_parameter('offset_exit').value
        self.smoothing_alpha = self.get_parameter('offset_smoothing_alpha').value
        self.max_yaw = self.get_parameter('max_yaw').value
        self.min_yaw_command = self.get_parameter('min_yaw_command').value
        self.offset_bias = self.get_parameter('offset_bias').value
        self.yaw_deadband = self.get_parameter('yaw_deadband').value

        self.kp_strafe = 0.5
        self.strafe_deadband = 0.08
        self.max_strafe = 0.20
        self.min_strafe_command = 0.0

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

    @staticmethod
    def soft_deadband(e, db):
        if abs(e) <= db:
            return 0.0
        return e - math.copysign(db, e)

    def compute_yaw(self, e):
        u = self.soft_deadband(e, self.yaw_deadband)
        v = -self.kp_yaw * u
        return max(-self.max_yaw, min(self.max_yaw, v))

    def compute_strafe(self, e):
        u = self.soft_deadband(e, self.strafe_deadband)
        v = -self.kp_strafe * u
        return max(-self.max_strafe, min(self.max_strafe, v))

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

    def reset_tracking(self):
        self.distance_state = 'STOP'
        self.is_turning = False
        self.filtered_offset = 0.0
        self.filter_ready = False
        self.last_cmd = (0.0, 0.0, 0.0)
        self.last_seen = None

    def control_loop(self):
        cmd = Twist()
        state = 'HILANG'
        now = self.get_clock().now()

        if self.target_locked and self.position is not None and self.position_is_fresh():
            offset_raw = self.position.x - self.offset_bias
            distance = self.position.y

            if not self.filter_ready:
                self.filtered_offset = offset_raw
                self.filter_ready = True
            else:
                a = self.smoothing_alpha
                self.filtered_offset = a * offset_raw + (1.0 - a) * self.filtered_offset

            if 0.0 < distance <= self.max_valid_distance:
                cmd.linear.x = self.compute_forward(distance)
                cmd.linear.y = self.compute_strafe(self.filtered_offset)
                cmd.angular.z = (self.compute_yaw(self.filtered_offset)
                                if self.enable_yaw else 0.0)

                self.last_cmd = (cmd.linear.x, cmd.linear.y, cmd.angular.z)
                self.last_seen = now
                state = (f'MENGIKUTI (jarak_state={self.distance_state}, '
                        f'strafe={cmd.linear.y:.2f}, yaw={cmd.angular.z:.2f})')

        elif self.last_seen is not None:
            age = (now - self.last_seen).nanoseconds * 1e-9
            if age < self.lost_hold:
                taper = min(self.lost_taper, self.lost_hold)
                full_until = self.lost_hold - taper
                f = 1.0 if age <= full_until else (self.lost_hold - age) / taper
                cmd.linear.x = self.last_cmd[0] * f
                cmd.linear.y = 0.0
                cmd.angular.z = self.last_cmd[2] * f
                state = 'HOLD (marker hilang sesaat)'
            else:
                self.reset_tracking()
        else:
            self.reset_tracking()

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