import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d  # 使用 open3d 替代 pcl
from std_msgs.msg import Header  # 导入 Header

class PCDPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, '/pointcloud', 10)

        # 加载 PCD 文件
        self.pcd_data = o3d.io.read_point_cloud("/home/spr/Sentry_ws/src/SPR-RM-Sentry/src/rm_localization/fast_lio/PCD/result1025.pcd")  # 替换为您 .pcd 文件的路径

        # 定时器设置为 1 秒
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # 提取点云数据
        points = [(point[0], point[1], point[2]) for point in self.pcd_data.points]
        print("11");

        # 创建 Header 对象
        header = Header()
        header.stamp = self.get_clock().now().to_msg()  # 设置时间戳
        header.frame_id = "map"  # 设置 frame_id

        # 创建 PointCloud2 消息
        pointcloud_msg = pc2.create_cloud_xyz32(header, points)

        # 发布点云消息
        self.publisher_.publish(pointcloud_msg)
        self.get_logger().info('Publishing point cloud data')

def main(args=None):
    rclpy.init(args=args)
    node = PCDPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

