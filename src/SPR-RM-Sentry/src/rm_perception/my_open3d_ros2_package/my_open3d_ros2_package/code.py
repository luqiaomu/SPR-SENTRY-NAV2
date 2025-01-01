import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LaserScanFilter(Node):
    def __init__(self):
        super().__init__('laserscan_filter')
        self.subscription = self.create_subscription(
            LaserScan,
            '/original_laserscan',
            self.laserscan_callback,
            10
        )
        self.publisher = self.create_publisher(
            LaserScan,
            '/filtered_laserscan',
            10
        )
        self.get_logger().info('LaserScan Filter node initialized')

        # 定义四边形区域的顶点坐标
        self.vertex1 = (1.0, 1.0)
        self.vertex2 = (3.0, 1.0)
        self.vertex3 = (3.0, 3.0)
        self.vertex4 = (1.0, 3.0)

        # 定义四阶变换矩阵
        self.transformation_matrix = np.array([
            [1, 0, 0, 1],  # 平移向量
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

    def laserscan_callback(self, msg):
        # 移除四边形区域内的点云
        filtered_ranges = self.remove_points_in_polygon(msg.ranges)

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

    def remove_points_in_polygon(self, ranges):
        filtered_ranges = []
        for i, range_value in enumerate(ranges):
            angle = msg.angle_min + i * msg.angle_increment

            # 将激光点变换到指定位置
            transformed_point = self.transform_point(range_value, angle)

            # 检查变换后的激光点是否在四边形内
            if self.point_in_polygon(transformed_point[0], transformed_point[1]):
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
        transformed_point = np.dot(self.transformation_matrix, homogeneous_point)

        # 返回变换后的点的二维坐标
        return transformed_point[:2]

    def point_in_polygon(self, x, y):
        # 判断点 (x, y) 是否在四边形内
        # 这里使用射线法判断点是否在多边形内部
        # 可以根据具体情况实现具体的算法
        # 这里只是一个简单的示例
        # 请根据你的实际需求进行修改
        if (self.vertex1[0] <= x <= self.vertex2[0] and 
            self.vertex1[1] <= y <= self.vertex3[1]):
            return True
        return False

def main(args=None):
    rclpy.init(args=args)
    laserscan_filter = LaserScanFilter()
    rclpy.spin(laserscan_filter)
    laserscan_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
