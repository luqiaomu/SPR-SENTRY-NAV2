import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import open3d as o3d
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import sensor_msgs.msg
import tf_transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



class RobotHandler(Node):
    def __init__(self):
        super().__init__('robot_handler')


           # Parameters for cmd_vel multiplier
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel_x_multiplier', 10)
        self.cmd_vel_multiplier = 3.0  # Default multiplier value
        
        self.target_frame = 'map'
        self.source_frame = 'livox_frame'
        self.robot_position = None  # 机器人当前位置






    # Parameters for laser scan handler
        self.laser_scan_subscription = self.create_subscription(
            LaserScan,
            '/original_scan', # 你的激光扫描主题名称
            self.laser_scan_callback,
            10)
        self.laser_scan_subscription

        self.laser_scan_publisher = self.create_publisher(LaserScan, '/scan', 10)
       

        self.declare_parameter('source_frame', 'livox_frame')             
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


        self.init_x = 0.0
        self.init_y = 0.0
        self.init_yaw = 0.0
        self.trans_init = np.eye(4)
        
        self.delete_polygons_vertices = [
        #    [
        #         [-2.51,2.86,1.57],
        #         [-0.21,2.96,1.57],
        #         [1.09,4.61,1.57],
        #         [-2.51,4.51,1.57]
        #     ],
           
        #     #l2   135
        #     [
        #         [3.14,-0.39,2.355],
        #         [4.94,-0.24,2.355],
        #         [6.59,-2.64,2.355],
        #         [4.54,-3.14,2.355]
        #     ],
        #     # l3    -135
        #     [
        #         [3.54,1.21,-2.355],
        #         [4.99,1.11,-2.355],
        #         [7.99,4.61,-2.355],
        #         [6.84,6.26,-2.355]
        #     ],
        # #     [
        #         [-3.43,0.66,1.57],
        #         [-1.13,0.66,1.57],
        #         [-0.28,2.26,1.57],
        #         [-3.18,2.61,1.57]
        #     ],
        #      # l1.5   180
        #     [
        #         [0.77,5.61,3.0],
        #         [0.47,4.36,3.0],
        #         [2.02,4.01,3.0],
        #         [2.17,5.06,3.0]
        #     ],
        #     #l2   135
        #     [
        #         [2.27,-3.09,2.355],
        #         [3.92,-2.54,2.355],
        #         [5.42,-4.79,2.355],
        #         [3.57,-5.24,2.355]
        #     ],
        #    # l3    -135
        #     [
        #         [2.47,-0.79,-2.355],
        #         [4.07,-1.14,-2.355],
        #         [6.92,2.61,-2.355],
        #         [5.82,3.81,-2.355]
        #     ],

        ]


    def laser_scan_callback(self, msg):
        self.get_logger().info("Received laser scan")
        # 移除四边形区域内的点云
        filtered_ranges = self.remove_points_in_polygon(msg.ranges, msg)

        # 创建新的 LaserScan 消息并填充数据
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.ranges = filtered_ranges

        # 发布修改后的 LaserScan 消息
        self.laser_scan_publisher.publish(filtered_msg)




    # def cmd_vel_callback(self, msg):
    #     # 获取机器人当前位置
    #     self.get_robot_position()

        # if self.robot_position:
        #     # 获取当前多边形的索引
        #     current_polygon_index = self.get_current_polygon_index()

        #     # 检查机器人是否在指定的多边形内
        #     if current_polygon_index is not None:
        #         current_polygon_vertices = self.delete_polygons_vertices[current_polygon_index]

        #         # 如果在多边形内，检查姿态是否在特定方向范围内
        #         if self.is_yaw_within_range(self.init_yaw, current_polygon_vertices):
        #             # 在指定的多边形内且姿态在特定方向范围内，则加速


        #             scaled_twist = Twist()
        #             scaled_twist.linear.x = 3.0
        #             scaled_twist.linear.y = msg.linear.y
        #             scaled_twist.linear.z = msg.linear.z
        #             scaled_twist.angular = msg.angular  # 保持角速度不变
        #             self.cmd_vel_publisher.publish(scaled_twist)
        #             self.get_logger().info("Accelerating within specified polygon and yaw range")
          
        #             return

        #     # 如果不满足加速条件，保持原速度不变
        #     self.cmd_vel_publisher.publish(msg)
        #     self.get_logger().info("Not accelerating within specified polygon or yaw range")




    # def get_current_polygon_index(self):
    #     # 获取机器人当前位置
    #     robot_position = self.robot_position
    #     if robot_position is None:
    #         return None
        
    #     # 检查机器人是否在任何一个多边形内部
    #     for index, polygon_vertices in enumerate(self.delete_polygons_vertices):
    #         if self.is_inside_any_polygon(robot_position, polygon_vertices):
    #             return index
        
    #     # 如果机器人不在任何一个多边形内部，返回 None
    #     return None






    # def cmd_vel_callback(self, msg):
    #      # 获取机器人当前位置
    #     self.get_robot_position()

 
    #     if self.robot_position:
    #         # 初始化加速状态
    #         should_accelerate = False

    #         # 获取当前多边形的索引
    #         current_polygon_index = self.get_current_polygon_index()

    #         # 检查机器人是否在指定的多边形内
    #         if current_polygon_index is not None:
    #             current_polygon_vertices = self.delete_polygons_vertices[current_polygon_index]

    #             # 如果在多边形内，检查姿态是否在特定方向范围内
    #             if self.is_yaw_within_range(self.init_yaw, current_polygon_vertices):
    #                 # 如果满足条件，则设置加速标志为True
    #                 should_accelerate = True

    #         # 如果机器人在指定的多边形内且姿态在特定方向范围内，则加速
    #         if should_accelerate:
    #             scaled_twist = Twist()
    #             scaled_twist.linear.x = 3.0
    #             scaled_twist.linear.y = msg.linear.y
    #             scaled_twist.linear.z = msg.linear.z
    #             scaled_twist.angular = msg.angular  # 保持角速度不变
    #             self.cmd_vel_publisher.publish(scaled_twist)
    #             self.get_logger().info("Accelerating within specified polygon and yaw range")
    #         else:
    #             # 否则，保持原速度不变
    #             self.cmd_vel_publisher.publish(msg)
    #             self.get_logger().info("Not accelerating within specified polygon or yaw range")










    def cmd_vel_callback(self, msg):
        # 获取机器人当前位置
        self.get_robot_position()


        if self.robot_position:
                # 初始化加速状态
                should_accelerate = False
                # for polygon_vertices in self.delete_polygons_vertices:
                #     # 检查机器人是否在特定的多边形内
                #     if self.is_inside_any_polygon(self.robot_position):
                #         # 如果在多边形内，检查姿态是否在特定方向范围内
                #         if self.is_yaw_within_range(self.init_yaw, polygon_vertices):  # 只使用当前多边形的方向
                #             # 如果满足两个条件，则设置加速标志为True
                #             should_accelerate = True
                #             break
                id=self.is_inside_any_polygon(self.robot_position)
                # 检查机器人是否在特定的多边形内
                if(id!=0):
                    # 如果在多边形内，检查姿态是否在特定方向范围内
                    if self.is_yaw_within_range(self.init_yaw, self.delete_polygons_vertices[id-1]):  # 只使用当前多边形的方向
                        # 如果满足两个条件，则设置加速标志为True
                        should_accelerate = True

                # 如果机器人在指定的多边形内且姿态在特定方向范围内，则加速
                if should_accelerate:
                    scaled_twist = Twist()
                    scaled_twist.linear.x = 2.5
                    scaled_twist.linear.y = msg.linear.y
                    scaled_twist.linear.z = msg.linear.z
                    scaled_twist.angular = msg.angular  # 保持角速度不变
                    self.cmd_vel_publisher.publish(scaled_twist)
                    print("1111111111111111111")
                    self.get_logger().info("Accelerating within specified polygon and yaw range")
                    
                elif id != 0 and not self.is_yaw_within_range(self.init_yaw, self.delete_polygons_vertices[id-1]):  # 只使用当前多边形的方向
                    scaled_twist = Twist()
                    scaled_twist.linear.x = 0.6
                    scaled_twist.linear.y = msg.linear.y
                    scaled_twist.linear.z = msg.linear.z
                    scaled_twist.angular = msg.angular  # 保持角速度不变
                    self.cmd_vel_publisher.publish(scaled_twist)
                else:
                    # 否则，保持原速度不变
                    self.cmd_vel_publisher.publish(msg)
                    self.get_logger().info("Not accelerating within specified polygon or yaw range")
                    print("2222222222222")










        # if self.robot_position:
        #     # 检查机器人是否在任何一个多边形内
        #     inside_polygon = False
        #     for polygon_vertices in self.delete_polygons_vertices:
        #         if self.is_inside_any_polygon(self.robot_position):
        #             # 如果在多边形内，检查姿态是否在当前多边形的方向范围内
        #             if self.is_yaw_within_range(self.init_yaw, polygon_vertices):
        #                 inside_polygon = True
        #                 break

            # # 如果机器人在多边形内且在范围内，则加速
            # if inside_polygon:
            #     scaled_twist = Twist()
            #     scaled_twist.linear.x = 3.0
            #     scaled_twist.linear.y = msg.linear.y
            #     scaled_twist.linear.z = msg.linear.z
            #     scaled_twist.angular = msg.angular  # 保持角速度不变
            #     self.cmd_vel_publisher.publish(scaled_twist)
            #     self.get_logger().info("Accelerating within polygon and yaw range")
            # else:
            #     # 否则，保持原速度不变
            #     self.cmd_vel_publisher.publish(msg)
            #     self.get_logger().info("Not accelerating within polygon or yaw range")





        # if self.robot_position:
        #         # 检查机器人是否在任何一个多边形内
        #         for polygon_vertices in self.delete_polygons_vertices:
        #             if self.is_inside_any_polygon(self.robot_position):
        #                 # 如果在多边形内，检查姿态是否在当前多边形的方向范围内
        #                 if self.is_yaw_within_range(self.init_yaw, polygon_vertices):
        #                     # 在多边形内，增加x方向速度
        #                     scaled_twist = Twist()
        #                     scaled_twist.linear.x = 3.0
        #                     scaled_twist.linear.y = msg.linear.y
        #                     scaled_twist.linear.z = msg.linear.z

        #                     scaled_twist.angular = msg.angular  # 保持角速度不变
        #                     self.cmd_vel_publisher.publish(scaled_twist)
        #                     self.get_logger().info("Accelerating within polygon and yaw range")
        #                     print("1111111111111111111")
        #                     return

        #     # 如果不在多边形内或姿态不在方向范围内，保持原速度不变
        # self.cmd_vel_publisher.publish(msg)
        # self.get_logger().info("Not accelerating within polygon or yaw range")
        # print("2222222222222")


            # if self.robot_position and self.is_inside_any_polygon(self.robot_position) and self.is_yaw_within_range(self.init_yaw):
        #     # 在多边形内，增加x方向速度
        #     scaled_twist = Twist()
        #     # scaled_twist.linear.x = msg.linear.x * self.multiplier
        #     scaled_twist.linear.x = 3.0
        #     scaled_twist.linear.y = msg.linear.y
        #     scaled_twist.linear.z = msg.linear.z

        #     scaled_twist.angular = msg.angular  # 保持角速度不变
        #     self.cmd_vel_publisher.publish(scaled_twist)
        #     print("Accelerating within polygon and yaw range")
        #     print("1111111111111111111")
        # else:
        #     # 不在多边形内，保持原速度不变
        #     self.cmd_vel_publisher.publish(msg)
        #     print("2222222222222")


    def get_robot_position(self):
        try:
            # 获取当前机器人位置
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,  # 目标坐标系
                'base_link',  # 源坐标系
                rclpy.time.Time())  # 当前时间
            self.robot_position = [trans.transform.translation.x, trans.transform.translation.y]
        except TransformException as ex:
            # 获取失败时，打印错误信息
            self.get_logger().info(f'Failed to get robot position: {ex}')
            self.robot_position = None


    # def remove_points_in_polygon(self, ranges, msg):
    #     filtered_ranges = []
    #     for i, range_value in enumerate(ranges):
    #         angle = msg.angle_min + i * msg.angle_increment

    #         # 将激光点变换到指定位置
    #         transformed_point = self.transform_point(range_value, angle)

    #         # 获取当前多边形的索引
    #         current_polygon_index = self.get_current_polygon_index()

    #         if current_polygon_index is not None:
    #             current_polygon_vertices = self.delete_polygons_vertices[current_polygon_index]
    #             # 检查变换后的激光点是否在四边形内
    #             if not self.is_inside_any_polygon(transformed_point, current_polygon_vertices):
    #                 filtered_ranges.append(range_value)
    #             else:
    #                 filtered_ranges.append(float('nan'))  # 不在区域内的点设置为 NaN
    #         else:
    #             filtered_ranges.append(range_value)

    #     return filtered_ranges


    def remove_points_in_polygon(self, ranges, msg):
        filtered_ranges = []
        for i, range_value in enumerate(ranges):
            angle = msg.angle_min + i * msg.angle_increment

            # 将激光点变换到指定位置
            transformed_point = self.transform_point(range_value, angle)

            id=self.is_inside_any_polygon(transformed_point)
            # 检查变换后的激光点是否在四边形内
            if(id==0):
                filtered_ranges.append(range_value)
            else:
                filtered_ranges.append(float('nan'))  # 不在区域内的点设置为 NaN

            # # 检查变换后的激光点是否在四边形内
            # if not self.is_inside_any_polygon(transformed_point):
            #     filtered_ranges.append(range_value)
            # else:
            #     filtered_ranges.append(float('nan'))  # 不在区域内的点设置为 NaN
        return filtered_ranges

    def transform_point(self, range_value, angle):
        # 将激光点表示为齐次坐标
        x = range_value * np.cos(angle)
        y = range_value * np.sin(angle)
        homogeneous_point = np.array([x, y, 0, 1])

        # 应用四阶变换矩阵
        transformed_point = np.dot(self.trans_init, homogeneous_point)

        # 返回变换后的点的二维坐标
        return transformed_point[:2]
   
    



    # def is_inside_any_polygon(self, point, polygon_vertices):
    #     # 检查点是否在任何一个四边形内部
    #     for polygon_vertex in polygon_vertices:
    #         # 从四边形的顶点列表中提取顶点坐标
    #         p1, p2, p3, p4 = polygon_vertex
    #         # 检查点是否在当前四边形内部
    #         if (self.get_cross(p1, p2, point) * self.get_cross(p3, p4, point) >= 0 and
    #             self.get_cross(p2, p3, point) * self.get_cross(p4, p1, point) >= 0):
    #             return True
    #     return False









    def is_inside_any_polygon(self, point):
        index=0
        add=0
        # 检查点是否在任何一个四边形内部
        for polygon_vertices in self.delete_polygons_vertices:
            add+=1
            # 从四边形的顶点列表中提取顶点坐标
            p1, p2, p3, p4 = polygon_vertices
            # 检查点是否在当前四边形内部
            if self.IsPointInMatrix(p1, p2, p3, p4, point):
                index=add
                break
        return index




    # def is_inside_any_polygon(self, point):
    #     index=0
    #     # 检查点是否在任何一个四边形内部
    #     for polygon_vertices in self.delete_polygons_vertices:
    #         # 从四边形的顶点列表中提取顶点坐标
    #         p1, p2, p3, p4 = polygon_vertices
    #         # 检查点是否在当前四边形内部
    #         if self.IsPointInMatrix(p1, p2, p3, p4, point):
                
    #             return True
          
              
    #     return False


    def is_yaw_within_range(self, yaw,polygon_vertices):
        # 检查姿态是否在指定范围内
            # 获取多边形的方向
        polygon_direction = polygon_vertices[0][2]  # 假设每个多边形的方向都一样，取第一个顶点的方向
            # 根据多边形的方向确定最小和最大偏航角
         # 计算小车方向与多边形方向之间的角度差
        # angle_diff = abs(yaw - polygon_direction)

        # 规范化角度
        normalized_yaw = self.normalize_angle(yaw)
        normalized_polygon_direction = self.normalize_angle(polygon_direction)
        # 计算小车方向与多边形方向之间的角度差
        angle_diff = abs(normalized_yaw - normalized_polygon_direction)


        self.get_logger().info(f"{angle_diff}")
            # 如果角度差不超过90度且不超过180度，返回True
        if angle_diff <= 1.57:
            return True
            
        # 如果姿态不在任何一个多边形的方向范围内，返回False
        return False
    


    def normalize_angle(self, angle):
        # 规范化角度到 [-pi, pi] 范围内
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
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
        self.get_logger().info('Get %s --> %s transform: [%f, %f, %f] [%f, %f, %f]' 
          % (self.source_frame, self.target_frame, pos.x, pos.y, pos.z, euler[0], euler[1], euler[2]))

        # Update initial position and yaw
        self.init_x = pos.x
        self.init_y = pos.y
        self.init_yaw = euler[2]

        self.trans_init = self.transformation_matrix(self.init_x, self.init_y, self.init_yaw)


    def transformation_matrix(self, x, y, yaw):
        Ro = self.euler_to_rotation_matrix(yaw)
        Tr = np.eye(4)
        Tr[:3, :3] = Ro
        Tr[0, 3] = x
        Tr[1, 3] = y
        return Tr

    def euler_to_rotation_matrix(self, yaw):
        Ro = np.eye(3)
        Ro[0, 0] = math.cos(yaw)
        Ro[0, 1] = -math.sin(yaw)
        Ro[1, 0] = math.sin(yaw)
        Ro[1, 1] = math.cos(yaw)
        return Ro
    
    def GetCross(self, p1, p2, p):
        return (p2[0] - p1[0]) * (p[1] - p1[1]) - (p[0] - p1[0]) * (p2[1] - p1[1])

    def IsPointInMatrix(self, p1, p2, p3, p4, p):
        isPointIn = self.GetCross(p1, p2, p) * self.GetCross(p3, p4, p) >= 0 and self.GetCross(p2, p3, p) * self.GetCross(p4, p1, p) >= 0
        return isPointIn
    
def main(args=None):
    rclpy.init(args=args)
    robot_handler = RobotHandler()
    rclpy.spin(robot_handler)
    robot_handler.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
