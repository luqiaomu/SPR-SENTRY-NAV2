#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

class LaserScanToPointCloud : public rclcpp::Node {
public:
  LaserScanToPointCloud() : Node("laser_scan_to_point_cloud") 
  { 
    double x_init, y_init, yaw_init;
    this->declare_parameter<double>("x_init", 0.0);    
    this->declare_parameter<double>("y_init", 0.0);
    this->declare_parameter<double>("yaw_init", 0.0);

    this->get_parameter_or<double>("x_init", x_init, 0.0);
    this->get_parameter_or<double>("y_init", y_init, 0.0);
    this->get_parameter_or<double>("yaw_init", yaw_init, 0.0);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/original_scan", 10, std::bind(&LaserScanToPointCloud::scanCallback, this, std::placeholders::_1));
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
     // 加载PCD地图
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/spr/Sentry_ws/src/SPR-RM-Sentry/src/rm_map/pgm_to_pointcloud/pcd/write1.pcd", *target_cloud_) == -1)  //write.pcd
    {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file");
      return;
    }

    // 创建一个kdtree用于目标点云
    
    kdtree.setInputCloud(target_cloud_);

    RCLCPP_INFO(this->get_logger(), "Loaded PCD map with %zu points", target_cloud_->points.size());

    // 初始化变换矩阵为单位矩阵
    previous_transformation_ = Eigen::Matrix4f::Identity();

    map_to_odom = transformMatrix(x_init, y_init, yaw_init);

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&LaserScanToPointCloud::listener_callback, this, std::placeholders::_1));

    // 初始化tf广播器
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // 获取开始时间点
    auto start = std::chrono::high_resolution_clock::now();
    sensor_msgs::msg::PointCloud2 cloud_msg;
    // laser_geometry::LaserProjection projector_;
    projector_.projectLaser(*scan, cloud_msg);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_msg, *input_cloud);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud = performICP(input_cloud, target_cloud_);
    if (aligned_cloud) {
      sensor_msgs::msg::PointCloud2 output;
      pcl::toROSMsg(*aligned_cloud, output);
      output.header.frame_id = "map";
      point_cloud_pub_->publish(output);

      map_to_odom = previous_transformation_ * odom_transformation_.inverse();

      Eigen::Vector3f map_to_odom_t = map_to_odom.block<3, 1>(0, 3);
      Eigen::Matrix3f map_to_odom_r = map_to_odom.block<3, 3>(0, 0);

      int32_t timestamp_offset = 0.21;  // 0.05
      int32_t timestamp_offset_sec = 0;
      uint32_t timestamp_offset_nanosec = 0;

      timestamp_offset_sec = static_cast<int>(timestamp_offset);
      timestamp_offset_nanosec = static_cast<int>((timestamp_offset - timestamp_offset_sec) * 1e9);

      int32_t new_sec = 0;
      uint32_t new_nanosec = 0;

      new_sec = scan->header.stamp.sec + timestamp_offset_sec; 
      new_nanosec = scan->header.stamp.nanosec + timestamp_offset_nanosec; 

      if (new_nanosec >= 1e9)
      {
          new_sec += 1;
          new_nanosec -= 1e9;
      }

      

      // 发布TF变换
      geometry_msgs::msg::TransformStamped transform_stamped;
      // transform_stamped.header.stamp = scan->header.stamp;
      transform_stamped.header.stamp.sec = new_sec; 
      transform_stamped.header.stamp.nanosec = new_nanosec; 
      transform_stamped.header.frame_id = "map";
      transform_stamped.child_frame_id = "odom";
      transform_stamped.transform.translation.x = map_to_odom_t.x();
      transform_stamped.transform.translation.y = map_to_odom_t.y();
      transform_stamped.transform.translation.z = map_to_odom_t.z();
      Eigen::Quaternionf q(map_to_odom_r);
      transform_stamped.transform.rotation.x = q.x();
      transform_stamped.transform.rotation.y = q.y();
      transform_stamped.transform.rotation.z = q.z();
      transform_stamped.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(transform_stamped);
    }
     // 获取结束时间点
    auto end = std::chrono::high_resolution_clock::now();

    // 计算时间差，单位为毫秒
    std::chrono::duration<double, std::milli> duration = end - start;

    RCLCPP_INFO(this->get_logger(), "Time taken: %f", duration.count());
  }


  pcl::PointCloud<pcl::PointXYZ>::Ptr performICP(
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) 
  {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);

    // 配置并行计算
    icp.setMaximumIterations(200); // 设置最大迭代次数
    icp.setTransformationEpsilon(1e-6); // 设置收敛条件
    icp.setEuclideanFitnessEpsilon(0.01); // 设置欧氏适应度收敛条件

    pcl::PointCloud<pcl::PointXYZ> Final;
    // icp.align(Final, previous_transformation_);
    icp.align(Final, map_to_odom * odom_transformation_);

    if (icp.hasConverged()) {
      std::cout << "ICP converged. Score: " << icp.getFitnessScore() << std::endl;

      // 更新上一次的变换矩阵
      previous_transformation_ = icp.getFinalTransformation();

      Eigen::Vector3f translation = previous_transformation_.block<3, 1>(0, 3);
      Eigen::Matrix3f rotation_matrix = previous_transformation_.block<3, 3>(0, 0);
      Eigen::Vector3f euler_angles = rotation_matrix.eulerAngles(2, 1, 0);

      euler_angles = euler_angles * (180 / M_PI); // 转换为角度制

      RCLCPP_INFO(this->get_logger(), "Translation: x=%f, y=%f", translation.x(), translation.y());
      RCLCPP_INFO(this->get_logger(), "Rotation: yaw=%f", euler_angles.x());

      

      pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud{new pcl::PointCloud<pcl::PointXYZ>};
      *aligned_cloud = Final;
      float overlap_rate = calculateOverlapRate(aligned_cloud, 0.1);
      RCLCPP_INFO(this->get_logger(), "Overlap rate: %f", overlap_rate);
      
      return Final.makeShared();
    } else {
      std::cout << "ICP did not converge." << std::endl;
      return nullptr;
    }
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

    Eigen::Matrix4f transformMatrix(double x, double y, double yaw) 
    {
      Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();

      // 计算yaw的cos和sin值
      double cos_yaw = std::cos(yaw);
      double sin_yaw = std::sin(yaw);

      // 填充4阶变换矩阵
      matrix(0, 0) = cos_yaw;
      matrix(0, 1) = -sin_yaw;
      matrix(0, 3) = x;

      matrix(1, 0) = sin_yaw;
      matrix(1, 1) = cos_yaw;
      matrix(1, 3) = y;

      return matrix;
    }

  float calculateOverlapRate(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    float distance_threshold)
  {
    

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

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  laser_geometry::LaserProjection projector_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_{new pcl::PointCloud<pcl::PointXYZ>};
  Eigen::Matrix4f previous_transformation_; // 保存上一次的变换矩阵
  Eigen::Matrix4f odom_transformation_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f map_to_odom = Eigen::Matrix4f::Identity();
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanToPointCloud>());
  rclcpp::shutdown();
  return 0;
}