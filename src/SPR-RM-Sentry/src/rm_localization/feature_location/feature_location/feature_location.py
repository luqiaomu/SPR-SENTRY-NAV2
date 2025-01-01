import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import open3d as o3d
from sensor_msgs_py import point_cloud2

class PointCloudSubscriber(Node):

    def __init__(self):
        super().__init__('pointcloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            'livox/lidar/pointcloud',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Load the PCD map
        self.pcd_map = o3d.io.read_point_cloud('/home/spr/Sentry_ws/src/SPR-RM-Sentry/src/rm_localization/fast_lio/PCD/result.pcd')

    def get_point_cloud(self, pc_msg):
        # Convert ROS PointCloud2 message to Open3D PointCloud
        points = np.frombuffer(pc_msg.data, dtype=np.float32).reshape(-1, pc_msg.point_step // 4)[:,:3]
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(points)
        return pc

    def listener_callback(self, msg):
        cloud = self.get_point_cloud(msg)
        
        
        # Perform registration
        self.perform_registration(cloud, self.pcd_map)

    def perform_registration(self, source, target):
        self.get_logger().info('Received a PointCloud2 message')
        # Perform down-sampling
        source_down = source.voxel_down_sample(voxel_size=0.05)
        target_down = target.voxel_down_sample(voxel_size=0.05)

        # Compute FPFH features
        radius_feature = 0.1

        self.get_logger().info('estimate_normals')

        source_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        target_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

        self.get_logger().info('fpfh_feature')

        source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            source_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            target_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        
        self.get_logger().info('ransac_registration')

         # Define RANSAC registration parameters
        max_correspondence_distance = 0.05
        estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPoint(False)
        ransac_n = 4
        checkers = [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                    o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(max_correspondence_distance)]
        criteria = o3d.pipelines.registration.RANSACConvergenceCriteria(400, 50)

        # Perform RANSAC registration
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,
            mutual_filter=True,
            max_correspondence_distance=max_correspondence_distance,
            estimation_method=estimation_method,
            ransac_n=ransac_n,
            checkers=checkers,
            criteria=criteria)

        self.get_logger().info('Registration result:\n%s' % str(result.transformation))


def main(args=None):
    rclpy.init(args=args)
    pointcloud_subscriber = PointCloudSubscriber()
    rclpy.spin(pointcloud_subscriber)
    pointcloud_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()