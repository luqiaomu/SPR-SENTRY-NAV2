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

class Spin_Controller(Node):
    def __init__(self):
        super().__init__('rm_controller')
        self.path_subscription = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.goal_subscription = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel_x_multiplier',
            self.cmd_vel_callback,
            10)
        
        self.spin_publisher = self.create_publisher(Twist, '/rm_cmd_vel', 10)

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/original_scan',  # 你的激光扫描主题名称
            self.laser_scan_callback,
            10)

        self.path_len = 0

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




    def path_callback(self, msg):
        self.path_len = len(msg.poses)
        if len(msg.poses) < self.forward:
            self.get_logger().info(f"Path contains less than {self.forward} points.")
        else:
            self.get_logger().info(f"Path length: {len(msg.poses)} points.")
            start_point = msg.poses[0].pose.position
            next_point = msg.poses[self.forward - 1].pose.position

            x = next_point.x - start_point.x
            y = next_point.y - start_point.y

            self.path_tangent_angle = math.atan2(y, x)

            # self.get_logger().info("Tanget angle at start point:{:.3f} radians".format(self.path_tangent_angle))

    def laser_scan_callback(self, msg):
        # self.get_logger().info("Received laser scan")
         # 假设激光雷达数据在水平方向上是均匀分布的
        angle_min = msg.angle_min  # 激光扫描的起始角度
        angle_increment = msg.angle_increment  # 激光扫描的角度增量
        ranges = msg.ranges  # 激光扫描的距离值列表

        # 设置正前方角度范围
        min_angle = -math.pi / 12  # 正前方左侧的角度
        max_angle = math.pi / 12  # 正前方右侧的角度

        # 计算正前方范围内的点云距离和分布范围
        front_distances = []
        for i in range(len(ranges)):
            angle = angle_min + i * angle_increment
            if min_angle <= angle <= max_angle:
                distance = ranges[i]
                if distance > 0 and distance < float('inf'):
                    front_distances.append(distance)

        # 计算正前方点云的最小、最大距离和平均距离
        min_distance = min(front_distances) if front_distances else float('inf')
        max_distance = max(front_distances) if front_distances else 0
        avg_distance = sum(front_distances) / len(front_distances) if front_distances else 0

        # 输出正前方点云的最小、最大距离和平均距离
        # print("Minimum distance in front:", min_distance)
        # print("Maximum distance in front:", max_distance)
        # print("Average distance in front:", avg_distance)
        

    def goal_callback(self, msg):
        # self.get_logger().info(f"Received goal:x={msg.pose.position.x},y={msg.pose.position.y}")
        if (msg.pose.position.x, msg.pose.position.y) != self.current_goal:
            self.current_goal = (msg.pose.position.x, msg.pose.position.y)
            self.start = True
            self.start_pos = self.last_pos
            # self.get_logger().info("start----------")
    
    def normalize_angle(self, angle):
        if angle < -math.pi:
            angle += 2 * math.pi
        if angle > math.pi:
            angle -= 2 * math.pi
        return angle

    def cmd_vel_callback(self, msg):
        if self.start and self.path_len > self.forward:
            self.angle_between_robot_and_path = self.normalize_angle(self.path_tangent_angle - self.robot_yaw)
            # self.get_logger().info("Angle:{:.3f} radians".format(self.angle_between_robot_and_path))
            self.get_logger().info('Start')
            if math.fabs(self.angle_between_robot_and_path) < self.yaw_tolerance:
                self.start = False
            else:
                if math.fabs(self.angle_between_robot_and_path) > self.decel_zone:
                    angular_z = self.max_theta_vel
                else:
                    angular_z_2 = 2 * self.acc * math.fabs(self.angle_between_robot_and_path) + self.min_theta_vel ** 2
                    if angular_z_2 > 0:
                        angular_z = math.sqrt(angular_z_2)
                    else:
                        angular_z = self.max_theta_vel

                if self.angle_between_robot_and_path < 0:
                    angular_z = -angular_z

                spin_twist = Twist()
                spin_twist.linear.x = 0.0
                spin_twist.linear.y = 0.0
                spin_twist.linear.z = 0.0
                spin_twist.angular.x = 0.0
                spin_twist.angular.y = 0.0
                spin_twist.angular.z = float(angular_z)
                
                self.spin_publisher.publish(spin_twist)
        elif self.stuck and self.path_len > self.forward:
            self.angle_between_robot_and_path = self.normalize_angle(self.path_tangent_angle - self.robot_yaw)
            self.get_logger().info('Stuck')
            if math.fabs(self.angle_between_robot_and_path) < self.yaw_tolerance:
                if self.acc_vel_time < 40:

                    acc_twist = Twist()
                    acc_twist.linear.x = 0.6
                    acc_twist.linear.y = 0.0
                    acc_twist.linear.z = 0.0
                    acc_twist.angular.x = 0.0
                    acc_twist.angular.y = 0.0
                    acc_twist.angular.z = 0.0
                    self.spin_publisher.publish(acc_twist)
                    self.acc_vel_time += 1
                else:
                    self.acc_vel_time = 0
                    self.stuck = False
                    self.stuck_time = 0.0
            else:
                if math.fabs(self.angle_between_robot_and_path) > self.decel_zone:
                    angular_z = self.max_theta_vel
                else:
                    angular_z_2 = 2 * self.acc * math.fabs(self.angle_between_robot_and_path) + self.min_theta_vel ** 2
                    if angular_z_2 > 0:
                        angular_z = math.sqrt(angular_z_2)
                    else:
                        angular_z = self.max_theta_vel

                if self.angle_between_robot_and_path < 0:
                    angular_z = -angular_z

                spin_twist = Twist()
                spin_twist.linear.x = 0.0
                spin_twist.linear.y = 0.0
                spin_twist.linear.z = 0.0
                spin_twist.angular.x = 0.0
                spin_twist.angular.y = 0.0
                spin_twist.angular.z = float(angular_z)
                # self.get_logger().info("vel:{:.3f} radians".format(angular_z))
                self.spin_publisher.publish(spin_twist)
        else:
            self.spin_publisher.publish(msg)

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
        # self.get_logger().info(f'Robot yaw angle:{euler[2]}')

        self.current_time = time.time()
        if self.last_time != None and self.last_pos != 0:
            d_t = self.current_time - self.last_time
            d_x = pos.x - self.last_pos.x
            d_y = pos.y - self.last_pos.y
            d_dist = (d_x ** 2 + d_y ** 2) ** 0.5
            vel = d_dist / d_t
            self.get_logger().info(f'Current velocity:{vel}')

            if self.start_pos != None:
                if self.path_len > 25 and vel < 0.3 and not self.start:
                    #  and math.sqrt((self.start_pos.x - pos.x) ** 2 + (self.start_pos.y - pos.y) ** 2) > 1.0
                    self.stuck_time += d_t
                else:
                    self.stuck_time = 0.0

            if self.stuck_time > 1.0 and not self.stuck:
                self.stuck = True

        self.last_time = self.current_time
        self.last_pos = pos
        
        



def main():
    rclpy.init()
    spin_controller = Spin_Controller()
    rclpy.spin(spin_controller)
    spin_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()