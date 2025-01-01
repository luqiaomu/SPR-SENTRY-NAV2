import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import open3d as o3d
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
class LaserScanHandler(Node):
    def __init__(self):
        super().__init__('laser_scan_handler')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # 你的激光扫描主题名称
            self.laser_scan_callback,
            10)
        self.subscription

        self.trans_init = np.eye(4)

        self.map = o3d.io.read_point_cloud("/home/lmh/Sentry_ws/src/SPR-RM-Sentry/src/commander/commander/pointcloud.pcd")

        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.vis.add_geometry(o3d.geometry.PointCloud())

    def laser_scan_callback(self, msg):
        # 将激光扫描数据转换为 PointCloud
        points = []
        for i, r in enumerate(msg.ranges):
            if r != float('inf'):
                angle = msg.angle_min + i * msg.angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0  # 如果没有 z 值，这里可以根据需要进行修改
                points.append([x, y, z])
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
     
        self.get_logger().info("Received laser scan")

       
        # o3d.visualization.draw_geometries([self.map, pcd])

        self.icp(pcd, self.map)
    
    def icp(self, source, target):
        source = source.voxel_down_sample(0.01)
        target = target.voxel_down_sample(0.01)
        source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        
        # 2. 点云配准
       
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source, target, 0.1, self.trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=10000)
        )
        
        reg_p_trans = reg_p2p.transformation

        # 3. 打印相对位姿变化
        translation_vector = reg_p_trans[:3, 3]
        rotation_matrix = reg_p_trans[:3, :3]
        quat = R.from_matrix(rotation_matrix.copy()).as_quat()
        # 欧拉角
        theta_x = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        theta_y = np.arctan2(-rotation_matrix[2, 0], np.sqrt(rotation_matrix[2, 1] ** 2 + rotation_matrix[2, 2] ** 2))
        theta_z = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        # 弧度转度数
        theta_x_deg = np.degrees(theta_x)
        theta_y_deg = np.degrees(theta_y)
        theta_z_deg = np.degrees(theta_z)
        # print("Transformation matrix:", np.round(reg_p_trans, 3))
        # print("Rotation matrix:", np.round(rotation_matrix, 3))
        print("Translation vector (x, y, z):", translation_vector)
        print("Euler Angle:\n\tx=%2f\n\ty=%2f\n\tz=%2f" %(theta_x_deg, theta_y_deg, theta_z_deg))
        # print("Quaternion:\n\tw=%2f\n\tx=%2f\n\ty=%2f\n\tz=%2f" %(quat[3], quat[0], quat[1], quat[2]))

        source.transform(reg_p_trans)

        self.trans_init = reg_p_trans

        # 更新 Open3D 可视化
        self.vis.clear_geometries()
        self.vis.add_geometry(source)
        self.vis.add_geometry(self.map)
        self.vis.poll_events()
        self.vis.update_renderer()

 
     
        # 构建KD树
        kdtree1 = o3d.geometry.KDTreeFlann(source)
        kdtree2 = o3d.geometry.KDTreeFlann(target)

        # 设置距离阈值
        distance_threshold = 0.01  # 设置最大距离阈值

        # 统计点云1中与点云2中点的距离小于阈值的点数
        overlap_points_count = 0
        for point in source.points:
            [_, idx, _] = kdtree2.search_knn_vector_3d(point, 1)
            closest_point = target.points[idx[0]]
            distance = np.linalg.norm(np.array(point) - np.array(closest_point))
            if distance < distance_threshold:
                overlap_points_count += 1

        # 计算重叠率
        overlap_ratio = overlap_points_count / len(source.points) * 100
        print("Overlap ratio:", overlap_ratio, "%")


def main(args=None):
    rclpy.init(args=args)
    laser_scan_handler = LaserScanHandler()
    rclpy.spin(laser_scan_handler)
    laser_scan_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
