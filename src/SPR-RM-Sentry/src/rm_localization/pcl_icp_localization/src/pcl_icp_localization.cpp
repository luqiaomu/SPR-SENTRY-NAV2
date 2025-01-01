#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <Eigen/Dense>  // 用于打印变换矩阵
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sstream>  // 用于打印变换矩阵
#include <cmath>
#include <omp.h>  // OpenMP支持
#include <chrono>
#include <thread>
#include <nav_msgs/msg/odometry.hpp>

class PointCloudICP : public rclcpp::Node
{
public:
  PointCloudICP() : Node("pointcloud_icp")
  {
    // 加载PCD地图
    if (pcl::io::loadPCDFile<pcl::PointXYZI>("/home/spr/Sentry_ws/src/SPR-RM-Sentry/src/rm_localization/fast_lio/PCD/result.pcd", *map_cloud_) == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file");
      return;
    }

    // 进行降采样
    pcl::VoxelGrid<pcl::PointXYZI> map_sor;
    map_sor.setInputCloud(map_cloud_);
    map_sor.setLeafSize(0.1f, 0.1f, 0.1f); // 根据需求设置叶子大小
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_map_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    map_sor.filter(*downsampled_map_cloud);
    map_cloud_ = downsampled_map_cloud;

    RCLCPP_INFO(this->get_logger(), "Loaded PCD map with %zu points", map_cloud_->points.size());

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar/pointcloud", 10, std::bind(&PointCloudICP::topic_callback, this, std::placeholders::_1));

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PointCloudICP::listener_callback, this, std::placeholders::_1));
    
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_pointcloud", 10);

    // 初始化变换矩阵为单位矩阵
    previous_transformation_ = Eigen::Matrix4f::Identity();
    // 初始化tf广播器
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
     // 获取开始时间点
    auto start = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZI>::Ptr incoming_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *incoming_cloud);

    // 进行降采样
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(incoming_cloud);
    sor.setLeafSize(0.1f, 0.1f, 0.1f); // 根据需求设置叶子大小
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    sor.filter(*downsampled_cloud);

    

    // ICP配准
    pcl::PointCloud<pcl::PointXYZI> aligned_cloud;
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;

    // 配置并行计算
    icp.setMaximumIterations(100); // 设置最大迭代次数
    icp.setTransformationEpsilon(1e-4); // 设置收敛条件
    icp.setEuclideanFitnessEpsilon(1); // 设置欧氏适应度收敛条件

    icp.setInputSource(downsampled_cloud);
    icp.setInputTarget(map_cloud_);
    
    // 使用上一次的变换结果作为初始猜测
    // icp.align(aligned_cloud, previous_transformation_);
    icp.align(aligned_cloud, odom_transformation_);

    if (icp.hasConverged())
    {
      RCLCPP_INFO(this->get_logger(), "ICP converged with score: %f", icp.getFitnessScore());
      
      // 更新上一次的变换矩阵
      previous_transformation_ = icp.getFinalTransformation();

      Eigen::Vector3f translation = previous_transformation_.block<3, 1>(0, 3);
      Eigen::Matrix3f rotation_matrix = previous_transformation_.block<3, 3>(0, 0);
      Eigen::Vector3f euler_angles = rotation_matrix.eulerAngles(2, 1, 0);

      euler_angles = euler_angles * (180 / M_PI); // 转换为角度制

      RCLCPP_INFO(this->get_logger(), "Translation: x=%f, y=%f, z=%f", translation.x(), translation.y(), translation.z());
      RCLCPP_INFO(this->get_logger(), "Rotation: roll=%f, pitch=%f, yaw=%f", euler_angles.z(), euler_angles.y(), euler_angles.x());

      // 发布TF变换
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped.header.stamp = msg->header.stamp;
      transform_stamped.header.frame_id = "map";
      transform_stamped.child_frame_id = "lidar_frame";
      transform_stamped.transform.translation.x = translation.x();
      transform_stamped.transform.translation.y = translation.y();
      transform_stamped.transform.translation.z = translation.z();
      Eigen::Quaternionf q(rotation_matrix);
      transform_stamped.transform.rotation.x = q.x();
      transform_stamped.transform.rotation.y = q.y();
      transform_stamped.transform.rotation.z = q.z();
      transform_stamped.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(transform_stamped);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
      return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud{new pcl::PointCloud<pcl::PointXYZI>};
    *transformed_cloud = aligned_cloud;

    // float overlap_rate = calculateOverlapRate(transformed_cloud, map_cloud_, 0.1);
    // RCLCPP_INFO(this->get_logger(), "Overlap rate: %f", overlap_rate);

    // 获取结束时间点
    auto end = std::chrono::high_resolution_clock::now();

    // 计算时间差，单位为毫秒
    std::chrono::duration<double, std::milli> duration = end - start;

    RCLCPP_INFO(this->get_logger(), "Time taken: %f", duration.count());

    // 发布配准后的点云
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(aligned_cloud, output);
    output.header = msg->header;
    publisher_->publish(output);
  }

  void listener_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract position and orientation
        auto pos = msg->pose.pose.position;
        auto ori = msg->pose.pose.orientation;
        double x = pos.x;
        double y = pos.y;
        double z = pos.z;
        double ox = ori.x;
        double oy = ori.y;
        double oz = ori.z;
        double ow = ori.w;

        // Convert quaternion to rotation matrix
        Eigen::Matrix3f rotation_matrix = quaternionToMatrix(ox, oy, oz, ow);

        
        odom_transformation_.block<3, 3>(0, 0) = rotation_matrix;
        odom_transformation_(0, 3) = x;
        odom_transformation_(1, 3) = y;
        odom_transformation_(2, 3) = z;
    }

    Eigen::Matrix3f quaternionToMatrix(double x, double y, double z, double w)
    {
        Eigen::Matrix3f matrix;
        double xx = x * x;
        double yy = y * y;
        double zz = z * z;
        double xy = x * y;
        double xz = x * z;
        double yz = y * z;
        double wx = w * x;
        double wy = w * y;
        double wz = w * z;

        matrix(0, 0) = 1 - 2 * (yy + zz);
        matrix(0, 1) = 2 * (xy - wz);
        matrix(0, 2) = 2 * (xz + wy);
        matrix(1, 0) = 2 * (xy + wz);
        matrix(1, 1) = 1 - 2 * (xx + zz);
        matrix(1, 2) = 2 * (yz - wx);
        matrix(2, 0) = 2 * (xz - wy);
        matrix(2, 1) = 2 * (yz + wx);
        matrix(2, 2) = 1 - 2 * (xx + yy);

        return matrix;
    }

  float calculateOverlapRate(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud,
    float distance_threshold)
  {
    // 创建一个kdtree用于目标点云
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(target_cloud);

    int overlap_count = 0;

    // 遍历源点云中的每个点
    for (const auto& point : source_cloud->points)
    {
        std::vector<int> point_idx_nkn_search(1);
        std::vector<float> point_nkn_squared_distance(1);

        // 在目标点云中查找该点的最近邻
        if (kdtree.nearestKSearch(point, 1, point_idx_nkn_search, point_nkn_squared_distance) > 0)
        {
            // 如果最近邻距离小于阈值，则认为是重合点
            if (point_nkn_squared_distance[0] < distance_threshold * distance_threshold)
            {
                overlap_count++;
            }
        }
    }

    // 计算重合率
    float overlap_rate = static_cast<float>(overlap_count) / source_cloud->points.size();
    return overlap_rate;
}

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_{new pcl::PointCloud<pcl::PointXYZI>};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  Eigen::Matrix4f previous_transformation_; // 保存上一次的变换矩阵
  Eigen::Matrix4f odom_transformation_ = Eigen::Matrix4f::Identity();
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  // 设置使用的线程数
  omp_set_num_threads(omp_get_max_threads());
  rclcpp::spin(std::make_shared<PointCloudICP>());
  rclcpp::shutdown();
  return 0;
}