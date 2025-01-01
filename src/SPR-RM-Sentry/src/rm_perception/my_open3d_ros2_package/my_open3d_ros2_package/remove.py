import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import open3d as o3d
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2


import sensor_msgs.msg

import tf_transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class LaserScanHandler(Node):
    def __init__(self):
        super().__init__('laser_scan_handler')
        self.subscription = self.create_subscription(
            LaserScan,
            '/original_scan',  # 你的激光扫描主题名称
            self.laser_scan_callback,
            10)
        self.subscription

        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

        self.trans_init = np.eye(4)

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

        self.delete_polygons_vertices = [
            #l1      顺次连接
            [
                [2.2, -0.8],
                [1.3, -2.7],
                [2.65, -3.0],
                [3.7, -1.0]
            ]
            #                                             [
            #     [-2.51,2.86],
            #     [-0.21,2.96],
            #     [1.09,4.61],
            #     [-2.51,4.51]
            # ],
            # #r1
            # [
            #     [17.29,-3.24],
            #     [19.34,-3.49],
            #     [19.19,-5.69],
            #     [14.79,-6.89]
            # ],
            #l2
            # [
            #     [3.14,-0.39],
            #     [4.94,-0.24],
            #     [6.59,-2.64],
            #     [4.54,-3.14]
            # ],
            #l3
            # [
            #     [3.54,1.21],
            #     [4.99,1.11],
            #     [7.99,4.61],
            #     [6.84,6.26]
            # ],
            # # #r2
            # [
            #     [11.89,-0.44],
            #     [13.59,-0.14],
            #     [12.34,2.31],
            #     [10.44,1.86]
            # ],
            # # #r3
            # [
            #     [11.84,-1.44],
            #     [13.64,-1.94],
            #     [10.29,-6.69],
            #     [8.84,-5.34]
            # ],

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
        self.publisher.publish(filtered_msg)

    def remove_points_in_polygon(self, ranges, msg):
        filtered_ranges = []
        for i, range_value in enumerate(ranges):
            angle = msg.angle_min + i * msg.angle_increment

            # 将激光点变换到指定位置
            transformed_point = self.transform_point(range_value, angle)

            # 检查变换后的激光点是否在四边形内
            if not self.is_inside_any_polygon(transformed_point):
                filtered_ranges.append(range_value)
            else:
                filtered_ranges.append(float('nan'))  # 不在区域内的点设置为 NaN
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
   
    
    def is_inside_any_polygon(self, point):
        # 检查点是否在任何一个四边形内部
        for polygon_vertices in self.delete_polygons_vertices:
            # 从四边形的顶点列表中提取顶点坐标
            p1, p2, p3, p4 = polygon_vertices
            # 检查点是否在当前四边形内部
            if self.IsPointInMatrix(p1, p2, p3, p4, point):
                return True
        return False

   

    

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
        # self.get_logger().info('Get %s --> %s transform: [%f, %f, %f] [%f, %f, %f]' 
        #   % (self.source_frame, self.target_frame, pos.x, pos.y, pos.z, euler[0], euler[1], euler[2]))

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
    laser_scan_handler = LaserScanHandler()
    rclpy.spin(laser_scan_handler)
    laser_scan_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
