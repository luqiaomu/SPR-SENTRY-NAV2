import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt
import time


class PathTrackerNode(Node):
    def __init__(self):
        super().__init__('path_tracker_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.subscription  # prevent unused variable warning
        self.current_position = None
        self.target_position = None
        self.current_yaw = None
        self.target_yaw = None
        self.linear_pid = PIDController(0.5, 0.1, 0.2)
        self.yaw_pid = PIDController(0.2, 0.05, 0.1)
        self.max_linear_velocity = 1.0
        self.max_angular_velocity = 2.0
        self.last_time = time.time()

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, self.current_yaw = self.euler_from_quaternion(orientation)

    def euler_from_quaternion(self, quat):
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w
        pitch = atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        roll = atan2(2 * (w * y - z * x), 1 - 2 * (y**2 + z**2))
        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (z**2 + y**2))
        return roll, pitch, yaw

    def compute_control(self):
        if self.current_position and self.target_position:
            error_linear = self.target_position.x - self.current_position.x
            
            # 仅当目标位置在当前位置的前方时才计算速度控制
            if error_linear > 0:
                linear_control = self.linear_pid.compute_control_output(error_linear, self.get_dt())
                linear_control = min(max(linear_control, 0), self.max_linear_velocity)
            else:
                linear_control = 0

            if self.current_yaw is not None and self.target_yaw is not None:
                error_yaw = self.target_yaw - self.current_yaw
                yaw_control = self.yaw_pid.compute_control_output(error_yaw, self.get_dt())
                yaw_control = min(max(yaw_control, -self.max_angular_velocity), self.max_angular_velocity)

                twist_msg = Twist()
                twist_msg.linear.x = linear_control
                twist_msg.angular.z = yaw_control

                self.publisher_.publish(twist_msg)

    def get_dt(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        return dt


class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.error_integral = 0

    def compute_control_output(self, error, dt):
        self.error_integral += error * dt
        error_derivative = (error - self.previous_error) / dt
        control_output = self.Kp * error + self.Ki * self.error_integral + self.Kd * error_derivative
        self.previous_error = error
        return control_output


def main(args=None):
    rclpy.init(args=args)
    path_tracker_node = PathTrackerNode()
    rclpy.spin(path_tracker_node)
    path_tracker_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()