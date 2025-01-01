#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import numpy as np
import sensor_msgs.msg
import tf_transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math


class Open3DPointCloudNode(Node):
    def __init__(self,init_x=0.0, init_y=0.0, init_yaw=0.0):
        super().__init__('open3d_pointcloud_node')
        
        self.subscription = self.create_subscription(
            PointCloud2,
            'livox/lidar/pointcloud',
            self.pointcloud_callback,
            1)
        self.subscription  # 防止Python清理时取消订阅
        
        self.publisher = self.create_publisher(PointCloud2, 'open3d_pointcloud', 10)

        # self.pc_visualizer = o3d.visualization.Visualizer()
        # self.pc_visualizer.create_window()



# Create a parameter for the source coordinate system name
        self.declare_parameter('source_frame', 'livox_frame')             
        self.source_frame = self.get_parameter(                     
            'source_frame').get_parameter_value().string_value
        # Create a parameter for the target coordinate system name
        self.declare_parameter('target_frame', 'map')             
        self.target_frame = self.get_parameter(                    
            'target_frame').get_parameter_value().string_value
        
        self.declare_parameter('init_x', 0.0)             
        self.init_x = self.get_parameter(                    
            'init_x').get_parameter_value().double_value
        
        self.declare_parameter('init_y', 0.0)             
        self.init_y = self.get_parameter(                    
            'init_y').get_parameter_value().double_value
        
        self.declare_parameter('init_yaw', 0.0)             
        self.init_yaw = self.get_parameter(                    
            'init_yaw').get_parameter_value().double_value


    # Create a buffer to save coordinate transformation information
        self.tf_buffer = Buffer()         
        # Create a listener for coordinate transformation
        self.tf_listener = TransformListener(self.tf_buffer, self)  
        # Create a fixed cycle timer to process coordinate information
        self.tf_timer = self.create_timer(1.0, self.tf_timer)   


        # 初始化参数
        self.init_x = init_x
        self.init_y = init_y
        self.init_yaw = init_yaw
             
             
       # 用于存储最新点云消息的变量
        self.latest_pointcloud_msg = None

      # 定义需要删除的多个四边形的顶点坐标
        self.delete_polygons_vertices = [
            #l1      顺次连接
            [
                [-2.51,2.86],
                [-0.21,2.96],
                [1.09,4.61],
                [-2.51,4.51]
            ],
            #r1
            # [
            #     [17.29,-3.24],
            #     [19.34,-3.49],
            #     [19.19,-5.69],
            #     [14.79,-6.89]
            # ],
            #l2
            [
                [3.14,-0.39],
                [4.94,-0.24],
                [6.59,-2.64],
                [4.54,-3.14]
            ],
            #l3
            [
                [3.54,1.21],
                [4.99,1.11],
                [7.99,4.61],
                [6.84,6.26]
            ],
            # #r2
            # [
            #     [11.89,-0.44],
            #     [13.59,-0.14],
            #     [12.34,2.31],
            #     [10.44,1.86]
            # ],
            # #r3
            # [
            #     [11.84,-1.44],
            #     [13.64,-1.94],
            #     [10.29,-6.69],
            #     [8.84,-5.34]
            # ],

        ]
    




    
    def pointcloud_callback(self, msg):
        self.get_logger().info('Received a PointCloud2 message')

        self.get_logger().info(f'{msg.height}')
        
        # # 更新最新的点云消息
        # self.latest_pointcloud_msg = msg
        
        # # Get the latest point cloud data
        # pc_msg = self.latest_pointcloud_msg

        pc_msg = msg


        pc = self.get_point_cloud(pc_msg) 
         # Call get_point_cloud with the latest point cloud message


         # 在筛选点云之前进行额外的变换
        trans = self.transformation_matrix(self.init_x, self.init_y, self.init_yaw)
        pc.transform(trans)


        # self.transform_point_cloud(pc)


        # 筛选出给定xy坐标范围内的点云数据
        filtered_pc = self.filter_point_cloud(pc) 

        inv_trans = np.linalg.inv(trans)
        filtered_pc.transform(inv_trans)

     
        # 可视化点云
        # self.visualize_pointcloud(filtered_pc)

        # 发布筛选后的点云数据
        self.publish_pointcloud(filtered_pc, msg)




    def get_point_cloud(self, pc_msg):
        # Convert ROS PointCloud2 message to Open3D PointCloud
        points = np.frombuffer(pc_msg.data, dtype=np.float32).reshape(-1, pc_msg.point_step // 4)[:,:3]
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)
        return pc



    
    def filter_point_cloud(self, pc):
        # 筛选出给定的多个四边形范围外的点云数据
        points = np.asarray(pc.points)
        filtered_indices = []

        # 遍历所有点，检查它们是否在任何一个四边形内部
        for i, point in enumerate(points):
            if not self.is_inside_any_polygon(point):
                filtered_indices.append(i)

        # 删除重复的索引并进行排序
        filtered_indices = np.unique(filtered_indices)

        # 删除在范围内的点
        filtered_pc = pc.select_by_index(filtered_indices)
        return filtered_pc






    def is_inside_any_polygon(self, point):
        # 检查点是否在任何一个四边形内部
        for polygon_vertices in self.delete_polygons_vertices:
            # 从四边形的顶点列表中提取顶点坐标
            p1, p2, p3, p4 = polygon_vertices
            # 检查点是否在当前四边形内部
            if self.IsPointInMatrix(p1, p2, p3, p4, point):
                return True
        return False


    def GetCross(self, p1, p2, p):
        return (p2[0] - p1[0]) * (p[1] - p1[1]) - (p[0] - p1[0]) * (p2[1] - p1[1])

    def IsPointInMatrix(self, p1, p2, p3, p4, p):
        isPointIn = self.GetCross(p1, p2, p) * self.GetCross(p3, p4, p) >= 0 and self.GetCross(p2, p3, p) * self.GetCross(p4, p1, p) >= 0
        return isPointIn





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


        self.position = pos
        self.robot_status = True
        # Update initial position and yaw
        self.init_x = pos.x
        self.init_y = pos.y
        self.init_yaw = euler[2]

        # 更新点云的坐标变换
        # 获取最新的点云消息
        # pc_msg = self.latest_pointcloud_msg
        # pc = self.get_point_cloud(pc_msg) 
        # Call get_point_cloud with the latest point cloud message


       # 在 transform_point_cloud 方法中进行坐标变换
        # self.transform_point_cloud(pc)

 
        


    def transform_point_cloud(self, pc):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                now)
            # Get rotation information
            quat = trans.transform.rotation
            euler = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

            # Create transformation matrix using x, y, and yaw
            trans_matrix = self.transformation_matrix(self.init_x, self.init_y, self.init_yaw)
            # Apply initial transformation应用初始变换

            pc.transform(trans_matrix)
           
           

            # Create transformation matrix from the received transform
            trans_matrix = self.transformation_matrix(trans.transform.translation.x,
                                                    trans.transform.translation.y,
                                                    euler[2])
            # Apply received transformation
            pc.transform(trans_matrix)

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform point cloud from {self.source_frame} to {self.target_frame}: {ex}')
        return pc





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





 # 创建一个新的 PointCloud2 消息对象
    def publish_pointcloud(self,pc,msg):
        open3d_pc_msg = sensor_msgs.msg.PointCloud2()
        
        # # 填充消息头信息
        open3d_pc_msg.header = msg.header

        # 将 Vector3dVector 转换为 numpy 数组
        pc_points = np.asarray(pc.points)
        
        # 定义字段 初始赋值
        open3d_pc_msg.height = msg.height
        open3d_pc_msg.width = len(pc_points)
        open3d_pc_msg.fields.append(sensor_msgs.msg.PointField(name="x", offset=0, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        open3d_pc_msg.fields.append(sensor_msgs.msg.PointField(name="y", offset=4, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        open3d_pc_msg.fields.append(sensor_msgs.msg.PointField(name="z", offset=8, datatype=sensor_msgs.msg.PointField.FLOAT32, count=1))
        open3d_pc_msg.is_bigendian = False
        open3d_pc_msg.point_step = 12
        open3d_pc_msg.row_step = open3d_pc_msg.point_step * len(pc_points)
        open3d_pc_msg.is_dense = False
        
        # 将点云数据转换为二进制格式并设置到消息中
        open3d_pc_msg.data = pc_points.astype(np.float32).tobytes()
        
        # # 发布点云消息
        self.publisher.publish(open3d_pc_msg)



    
        # 可视化点云
    # def visualize_pointcloud(self, pc):
    #     self.pc_visualizer.clear_geometries()
    #     self.pc_visualizer.add_geometry(pc)
    #     self.pc_visualizer.poll_events()
    #     self.pc_visualizer.update_renderer()
        
        
        
    

def main(args=None):
    rclpy.init(args=args)



# 创建节点并传入节点选项对象
    open3d_node = Open3DPointCloudNode(init_x=0.0, init_y=0.0, init_yaw=0.0)
    # open3d_node.create_timer(0.1, open3d_node.tf_timer)

    rclpy.spin(open3d_node)
    open3d_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

