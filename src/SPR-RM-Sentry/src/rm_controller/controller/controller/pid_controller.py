from typing import List
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import tf_transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time

class PID_Controller(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.path_subscription = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.goal_subscription = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel_x_multiplier',
            self.cmd_vel_callback,
            10)
        
        self.vel_publisher = self.create_publisher(Twist, '/pid_cmd_vel', 10)

        self.path_len = 0
        self.path = None

        self.current_goal = ()

        self.start = False

        self.stuck = False
        self.stuck_time = 0.0

        self.path_tangent_angle = 0.0

        self.robot_yaw = 0.0

        self.angle_between_robot_and_path = 0.0

        self.current_time = 0.0
        self.last_time = None
        self.last_pos = None 
        self.start_pos = None

        self.acc_vel_time = 0
        

        self.declare_parameter('forward', 20)             
        self.forward = self.get_parameter(                    
            'forward').get_parameter_value().integer_value

        self.declare_parameter('yaw_tolerance', 0.2)             
        self.yaw_tolerance = self.get_parameter(                    
            'yaw_tolerance').get_parameter_value().double_value
        
        self.declare_parameter('acc', 1.5)             
        self.acc = self.get_parameter(                    
            'acc').get_parameter_value().double_value
        
        self.declare_parameter('max_theta_vel',1.2)             
        self.max_theta_vel = self.get_parameter(                    
            'max_theta_vel').get_parameter_value().double_value
        
        self.declare_parameter('min_theta_vel', 0.2)             
        self.min_theta_vel = self.get_parameter(                    
            'min_theta_vel').get_parameter_value().double_value
        
        self.decel_zone = (((self.max_theta_vel - self.min_theta_vel) / self.acc) * (self.max_theta_vel + self.min_theta_vel)) / 2

        self.get_logger().info(f"{self.decel_zone}")

        # Create a parameter for the source coordinate system name
        self.declare_parameter('source_frame', 'base_link')             
        self.source_frame = self.get_parameter(                     
            'source_frame').get_parameter_value().string_value
        # Create a parameter for the target coordinate system name
        self.declare_parameter('target_frame', 'map')             
        self.target_frame = self.get_parameter(                    
            'target_frame').get_parameter_value().string_value
        
         # Create a buffer to save coordinate transformation information
        self.tf_buffer = Buffer()         
        # Create a listener for coordinate transformation
        self.tf_listener = TransformListener(self.tf_buffer, self)  
        # Create a fixed cycle timer to process coordinate information
        self.tf_timer = self.create_timer(0.1, self.tf_timer)   

        self.controller_timer = self.create_timer(0.02, self.compute_control)


        self.current_position = None
        self.target_position = None
        self.current_yaw = None
        self.target_yaw = None
        self.linear_pid = PIDController(0.5, 0.1, 0.2)
        self.yaw_pid = PIDController(0.2, 0.05, 0.1)
        self.max_linear_velocity = 1.0
        self.max_angular_velocity = 2.0
        self.last_time = time.time()

        self.is_control = False


    def path_callback(self, msg):
        self.path_len = len(msg.poses)
        self.path = msg.poses
        if len(msg.poses) < self.forward:
            self.get_logger().info(f"Path contains less than {self.forward} points.")
        else:
            self.get_logger().info(f"Path length: {len(msg.poses)} points.")
            

            # self.get_logger().info("Tanget angle at start point:{:.3f} radians".format(self.path_tangent_angle))

    def compute_control(self):
        if self.is_control:
            if len(self.path) > self.forward:
                self.target_position = self.path[self.forward - 1].pose.position
                start_point = self.path.poses[0].pose.position
                next_point = self.path.poses[self.forward - 1].pose.position

                x = next_point.x - start_point.x
                y = next_point.y - start_point.y


                self.target_yaw = self.normalize_angle(math.atan2(y, x))

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

                    self.vel_publisher.publish(twist_msg)


    def get_dt(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        return dt
        

    def goal_callback(self, msg):
        # self.get_logger().info(f"Received goal:x={msg.pose.position.x},y={msg.pose.position.y}")
        if (msg.pose.position.x, msg.pose.position.y) != self.current_goal:
            self.current_goal = (msg.pose.position.x, msg.pose.position.y)
            self.start = True
            self.is_control = True
            self.start_pos = self.last_pos
            # self.get_logger().info("start----------")
    
    def normalize_angle(self, angle):
        if angle < -math.pi:
            angle += 2 * math.pi
        if angle > math.pi:
            angle -= 2 * math.pi
        return angle

    

     # Tf listening callback(between map and base_link)
    def tf_timer(self):
        try:
            # Obtain the current time of the ROS system
            now = rclpy.time.Time()   
            # Monitor the coordinate transformation from the source coordinate system to the target coordinate system at the current time                          
            trans = self.tf_buffer.lookup_transform(                
                self.target_frame,
                self.source_frame,
                now)
        # If coordinate transformation acquisition fails, enter an exception report
        except TransformException as ex:                            
            self.get_logger().info(
                f'Could not transform {self.target_frame} to {self.source_frame}: {ex}')
            return
        # Obtain location information
        pos  = trans.transform.translation           
        # Obtain posture information (quaternion)
        quat = trans.transform.rotation                             
        euler = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.robot_yaw = euler[2]
        self.current_yaw = euler[2]
        self.current_position = pos
        # self.get_logger().info(f'Robot yaw angle:{euler[2]}')


        
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



def main():
    rclpy.init()
    spin_controller = PID_Controller()
    rclpy.spin(spin_controller)
    spin_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()