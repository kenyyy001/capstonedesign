import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Point, Twist


class FollowControllerNode(Node):
    def __init__(self):
        super().__init__('follow_controller_node')

        defaults = {
            'kp_yaw': 1.0, 'max_yaw': 0.6, 'yaw_deadband': 0.10,
            'enable_yaw': False,
            'kp_strafe': 0.5, 'strafe_deadband': 0.08, 'max_strafe': 0.20,
            'offset_smoothing_alpha': 0.3, 'offset_bias': 0.0,
            'desired_distance_m': 1.0, 'distance_band_m': 0.15,
            'forward_speed': 0.35, 'backward_speed': 0.30,
            'max_valid_distance_m': 5.0,
            'lost_timeout_s': 0.2,
            'coast_s': 0.4, 'settle_s': 0.4,
            'search_enable': True, 'search_yaw': 0.25,
            'search_pulse_s': 0.3, 'search_pause_s': 0.6,
            'search_max_s': 5.0,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        gp = lambda k: self.get_parameter(k).value

        self.kp_yaw = gp('kp_yaw')
        self.max_yaw = gp('max_yaw')
        self.yaw_deadband = gp('yaw_deadband')
        self.enable_yaw = gp('enable_yaw')
        self.kp_strafe = gp('kp_strafe')
        self.strafe_deadband = gp('strafe_deadband')
        self.max_strafe = gp('max_strafe')
        self.smoothing_alpha = gp('offset_smoothing_alpha')
        self.offset_bias = gp('offset_bias')
        self.desired = gp('desired_distance_m')
        self.band = gp('distance_band_m')
        self.forward_speed = gp('forward_speed')
        self.backward_speed = gp('backward_speed')
        self.max_valid_distance = gp('max_valid_distance_m')
        self.lost_timeout = gp('lost_timeout_s')
        self.coast_s = gp('coast_s')
        self.settle_s = gp('settle_s')
        self.search_enable = gp('search_enable')
        self.search_yaw = gp('search_yaw')
        self.search_pulse = gp('search_pulse_s')
        self.search_pause = gp('search_pause_s')
        self.search_max = gp('search_max_s')

        self.position = None
        self.last_position_time = None
        self.new_sample = False
        self.last_state = None
        self.last_cmd = (0.0, 0.0, 0.0)
        self.last_seen = None
        self.last_side = 1.0          # sisi terakhir marker terlihat (+ = kanan)
        self.filtered_offset = 0.0
        self.filter_ready = False
        self.distance_state = 'STOP'

        self.create_subscription(Point, '/vision/target_position',
                                 self.position_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(1.0 / 20.0, self.control_loop)

        self.get_logger().info(
            f'Follow controller siap. Jarak target {self.desired:.2f} m '
            f'(maju>{self.desired + self.band:.2f}, mundur<{self.desired - self.band:.2f})')

    def position_callback(self, msg):
        self.position = msg
        self.last_position_time = self.get_clock().now()
        self.new_sample = True

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
        v = -self.kp_yaw * self.soft_deadband(e, self.yaw_deadband)
        return max(-self.max_yaw, min(self.max_yaw, v))

    def compute_strafe(self, e):
        v = -self.kp_strafe * self.soft_deadband(e, self.strafe_deadband)
        return max(-self.max_strafe, min(self.max_strafe, v))

    def compute_forward(self, dist):
        d, b, st = self.desired, self.band, self.distance_state
        if dist > d + b:
            st = 'FORWARD'
        elif dist < d - b:
            st = 'BACKWARD'
        elif st == 'FORWARD' and dist <= d + 0.3 * b:
            st = 'STOP'
        elif st == 'BACKWARD' and dist >= d - 0.3 * b:
            st = 'STOP'
        self.distance_state = st
        if st == 'FORWARD':
            return self.forward_speed
        if st == 'BACKWARD':
            return -self.backward_speed
        return 0.0

    def reset_tracking(self):
        self.distance_state = 'STOP'
        self.filtered_offset = 0.0
        self.filter_ready = False
        self.last_cmd = (0.0, 0.0, 0.0)
        self.last_seen = None

    def control_loop(self):
        cmd = Twist()
        state = 'HILANG'
        now = self.get_clock().now()

        # Kesegaran posisi menggantikan target_locked (yang turun tiap 1 frame hilang)
        seen = (self.position is not None and self.position_is_fresh()
                and 0.0 < self.position.y <= self.max_valid_distance)

        if seen:
            if self.new_sample:
                self.new_sample = False
                raw = self.position.x - self.offset_bias
                if not self.filter_ready:
                    self.filtered_offset = raw
                    self.filter_ready = True
                else:
                    a = self.smoothing_alpha
                    self.filtered_offset = a * raw + (1.0 - a) * self.filtered_offset

            if abs(self.filtered_offset) > 0.15:
                self.last_side = math.copysign(1.0, self.filtered_offset)

            cmd.linear.x = self.compute_forward(self.position.y)
            cmd.linear.y = self.compute_strafe(self.filtered_offset)
            cmd.angular.z = (self.compute_yaw(self.filtered_offset)
                             if self.enable_yaw else 0.0)

            self.last_cmd = (cmd.linear.x, cmd.linear.y, cmd.angular.z)
            self.last_seen = now
            state = (f'MENGIKUTI (jarak={self.distance_state}, '
                     f'strafe={cmd.linear.y:.2f}, yaw={cmd.angular.z:.2f})')

        elif self.last_seen is not None:
            age = (now - self.last_seen).nanoseconds * 1e-9
            t_settle = self.coast_s + self.settle_s

            if age < self.coast_s:
                f = 1.0 - age / self.coast_s
                reversing = self.last_cmd[0] < 0.0
                # mundur = buta ke belakang -> berhenti langsung
                cmd.linear.x = 0.0 if reversing else self.last_cmd[0] * f
                cmd.linear.y = self.last_cmd[1] * f
                cmd.angular.z = self.last_cmd[2] * f
                state = 'COAST (mundur: stop)' if reversing else 'COAST'
            elif age < t_settle:
                self.filter_ready = False
                state = 'SETTLE (diam)'
            elif self.search_enable and age < t_settle + self.search_max:
                t = age - t_settle
                phase = t % (self.search_pulse + self.search_pause)
                if phase < self.search_pulse:
                    # konvensi sama dgn compute_yaw: offset + (kanan) -> yaw negatif
                    cmd.angular.z = -self.last_side * self.search_yaw
                    state = 'SEARCH (putar)'
                else:
                    state = 'SEARCH (diam, deteksi)'
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