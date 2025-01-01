import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import open3d as o3d
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import PointCloud2
import tf_transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
class CmdVelMultiplier(Node):



    def __init__(self):
        super().__init__('cmd_vel_x_multiplier')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel_x_multiplier', 10)
        self.multiplier = 3.0  # 设置倍数，默认为2
        
   

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_frame = 'map'
        self.source_frame = 'livox_frame'


        self.init_x = 0.0
        self.init_y = 0.0
        self.init_yaw = 0.0

        self.robot_position = None  # 机器人当前位置

        self.delete_polygons_vertices = [
            #l1      顺次连接
            [
                [2.2, -0.8],
                [1.3, -2.7],
                [2.65, -3.0],
                [3.7, -1.0]
            ]
            #l1
            # [
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
            # #l2
            # [
            #     [3.14,-0.39],
            #     [4.94,-0.24],
            #     [6.59,-2.64],
            #     [4.54,-3.14]
            # ],
            # #l3
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
        self.timer_period = 0.1
        self.tf_timer = self.create_timer(self.timer_period, self.tf_timer)


        # self.tf_timer = self.create_timer(1.0, self.tf_timer)
        # self.tf_timer_cancelled = False  # 标志位，表示是否取消了tf定时器


    def cmd_vel_callback(self, msg):
     
  # 获取机器人当前位置
        self.get_robot_position()

        if self.robot_position and self.is_inside_any_polygon(self.robot_position):
            # 在多边形内，增加x方向速度
            scaled_twist = Twist()
            # scaled_twist.linear.x = msg.linear.x * self.multiplier
            scaled_twist.linear.x = 3.0
            scaled_twist.linear.y = msg.linear.y
            scaled_twist.linear.z = msg.linear.z

            scaled_twist.angular = msg.angular  # 保持角速度不变
            # scaled_twist.angular.x = msg.angular.x
            # scaled_twist.angular.y = msg.angular.y
            # scaled_twist.angular.z = msg.angular.z
            self.publisher.publish(scaled_twist)
            print("1111111111111111111")
        else:
            # 不在多边形内，保持原速度不变
            self.publisher.publish(msg)


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





    # Tf listening callback(between map and base_link)
    def tf_timer(self):
        try:
            # Obtain the current time of the ROS system
            # now = rclpy.time.Time()   
            # Monitor the coordinate transformation from the source coordinate system to the target coordinate system at the current time                          
            trans = self.tf_buffer.lookup_transform(                
                self.target_frame,
                self.source_frame,
                rclpy.time.Time())

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

        except TransformException as ex:
            if 'Lookup would require extrapolation into the past' in str(ex):
            # TF2缓冲区中没有足够的信息，稍后再尝试
                self.get_logger().info(f'Waiting for transform information: {ex}')
            else:
            # 其他类型的TransformException错误
                self.get_logger().info(f'Could not transform {self.source_frame} to {self.target_frame}: {ex}')
            return


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
    


    def is_inside_any_polygon(self, point):
        # 检查点是否在任何一个四边形内部
        for polygon_vertices in self.delete_polygons_vertices:
            # 从四边形的顶点列表中提取顶点坐标
            p1, p2, p3, p4 = polygon_vertices
            # 检查点是否在当前四边形内部
            if self.IsPointInMatrix(p1, p2, p3, p4, point):
                print("222222")
                return True
        print("3333333")     
        return False

    def GetCross(self, p1, p2, p):
        return (p2[0] - p1[0]) * (p[1] - p1[1]) - (p[0] - p1[0]) * (p2[1] - p1[1])
    
    def IsPointInMatrix(self, p1, p2, p3, p4, p):
        isPointIn = self.GetCross(p1, p2, p) * self.GetCross(p3, p4, p) >= 0 and self.GetCross(p2, p3, p) * self.GetCross(p4, p1, p) >= 0
        return isPointIn
    




def main(args=None):
    rclpy.init(args=args)
    cmd_vel_multiplier = CmdVelMultiplier()
    rclpy.spin(cmd_vel_multiplier)
    cmd_vel_multiplier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
