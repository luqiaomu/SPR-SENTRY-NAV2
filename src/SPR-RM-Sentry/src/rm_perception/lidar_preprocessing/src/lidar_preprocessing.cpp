#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <Eigen/Dense> // Eigen 库用于矩阵运算

class PointCloudFilter : public rclcpp::Node {
public:
  PointCloudFilter() : Node("lidar_preprocessing") {
    // 初始化旋转矩阵
    initialize_rotation_matrices();

    // 订阅 Livox 自定义点云消息
    pointcloud_subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        "/livox/lidar", 10, std::bind(&PointCloudFilter::pointcloud_callback, this, std::placeholders::_1));

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/livox/lidar/pointcloud", 10, std::bind(&PointCloudFilter::topic_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar/pointcloud_ros", 10);

    // 订阅 IMU 数据
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10, std::bind(&PointCloudFilter::imu_callback, this, std::placeholders::_1));

    // 发布过滤后的点云
    pointcloud_publisher_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("/filtered_pointcloud", 10);

    // 发布翻转后的 IMU 数据
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/livox/imu/filtered", 10);
  }

private:
  void initialize_rotation_matrices() {
    // 定义旋转角度
    const double theta_x =  -1.185* M_PI / 180.0; // 绕 X 轴倾斜1.2
    const double theta_y = -1.9* M_PI / 180.0; // 绕 Y 轴倾斜1.2
    const double theta_z = 0.00 * M_PI / 180.0; // 绕 Z 轴旋转

    // **绕 X 轴旋转 180° 的矩阵**
    rotation_180_x_ << 1,  0,  0,
                       0, -1,  0,
                       0,  0, -1;

    // **绕 X 轴倾斜的旋转矩阵**
    rotation_x_ << 1, 0, 0,
                   0, cos(theta_x), -sin(theta_x),
                   0, sin(theta_x), cos(theta_x);

    // **绕 Y 轴倾斜的旋转矩阵**
    rotation_y_ << cos(theta_y), 0, sin(theta_y),
                   0, 1, 0,
                   -sin(theta_y), 0, cos(theta_y);

    // **绕 Z 轴旋转的矩阵**
    rotation_z_ << cos(theta_z), -sin(theta_z), 0,
                   sin(theta_z), cos(theta_z), 0,
                   0, 0, 1;

    // 组合旋转矩阵
    rotation_matrix_ = rotation_180_x_ * rotation_y_ * rotation_x_ * rotation_z_;
  }

  void pointcloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (msg->points.empty()) {
      RCLCPP_WARN(this->get_logger(), "No points received, skipping processing.");
      return;
    }

    // 从 Livox 自定义消息中提取点云数据并应用旋转矩阵
    for (const auto &point : msg->points) {
      pcl::PointXYZI pcl_point;
      Eigen::Vector3d original_point(point.x, point.y, point.z);

      // 应用旋转矩阵
      Eigen::Vector3d rotated_point = rotation_matrix_ * original_point;

      pcl_point.x = rotated_point.x();
      pcl_point.y = rotated_point.y();
      pcl_point.z = rotated_point.z();
      pcl_point.intensity = point.reflectivity;

      pcl_cloud->points.push_back(pcl_point);
    }

    // 使用 CropBox 过滤器
    pcl::CropBox<pcl::PointXYZI> crop_box_filter;
    crop_box_filter.setInputCloud(pcl_cloud);

    Eigen::Vector4f min_point(-0.290, -0.30, -0.30, 1.0);   //-0.275, -0.30, -0.170, 1.0
    Eigen::Vector4f max_point(0.290, 0.30, 0.530, 1.0);     //0.275, 0.30, 0.530, 1.0
    crop_box_filter.setMin(min_point);
    crop_box_filter.setMax(max_point);
    crop_box_filter.setNegative(true);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    crop_box_filter.filter(*filtered_cloud);

    // 构造 Livox 自定义消息，保留原始字段
    livox_ros_driver2::msg::CustomMsg output_msg = *msg;
    output_msg.point_num = filtered_cloud->points.size();
    output_msg.points.clear();

    for (size_t i = 0; i < filtered_cloud->points.size(); ++i) {
      livox_ros_driver2::msg::CustomPoint point;
      point.x = filtered_cloud->points[i].x;
      point.y = filtered_cloud->points[i].y;
      point.z = filtered_cloud->points[i].z;
      point.reflectivity = filtered_cloud->points[i].intensity;

      // 保留原始的 offset_time、tag 和 line
      point.offset_time = msg->points[i].offset_time;
      point.tag = msg->points[i].tag;
      point.line = msg->points[i].line;

      output_msg.points.push_back(point);
    }

    pointcloud_publisher_->publish(output_msg);
    RCLCPP_INFO(this->get_logger(), "Filtered and flipped point cloud published with %zu points", filtered_cloud->points.size());
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    sensor_msgs::msg::Imu flipped_imu = *msg;

    // 使用旋转矩阵翻转 IMU 数据
    Eigen::Vector3d linear_acceleration(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z);
    linear_acceleration = rotation_180_x_ * linear_acceleration;  //rotation_matrix

    flipped_imu.linear_acceleration.x = linear_acceleration.x();
    flipped_imu.linear_acceleration.y = linear_acceleration.y();
    flipped_imu.linear_acceleration.z = linear_acceleration.z();

    Eigen::Vector3d angular_velocity(
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z);
    angular_velocity = rotation_180_x_ * angular_velocity;  //rotation_matrix

    flipped_imu.angular_velocity.x = angular_velocity.x();
    flipped_imu.angular_velocity.y = angular_velocity.y();
    flipped_imu.angular_velocity.z = angular_velocity.z();

    Eigen::Quaterniond orientation(
        msg->orientation.w,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z);
    orientation = Eigen::Quaterniond(rotation_180_x_) * orientation;  //rotation_matrix

    flipped_imu.orientation.w = orientation.w();
    flipped_imu.orientation.x = orientation.x();
    flipped_imu.orientation.y = orientation.y();
    flipped_imu.orientation.z = orientation.z();

    imu_publisher_->publish(flipped_imu);
    RCLCPP_INFO(this->get_logger(), "Flipped IMU data published.");
  }
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(*msg, *pcl_cloud);

      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      for (const auto &point : pcl_cloud->points) {
          // 翻转 y 和 z 轴（180 度等价于取负值）
          pcl::PointXYZI flipped_point = point;
          flipped_point.y = -point.y;
          flipped_point.z = -point.z;

          // 添加距离过滤条件
          if (sqrt(flipped_point.x * flipped_point.x + flipped_point.y * flipped_point.y + flipped_point.z * flipped_point.z) > 0.5) {
              filtered_cloud->points.push_back(flipped_point);
          }
      }

      // 转换回 ROS 消息并发布
      sensor_msgs::msg::PointCloud2 output;
      pcl::toROSMsg(*filtered_cloud, output);
      output.header = msg->header;
      publisher_->publish(output);

      RCLCPP_INFO(this->get_logger(), "Filtered and flipped point cloud published with %zu points", filtered_cloud->points.size());
  }

  // void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  // {
  //   pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  //   pcl::fromROSMsg(*msg, *pcl_cloud);

  //   pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  //   for (const auto& point : pcl_cloud->points) {
  //     if (sqrt(point.x * point.x + point.y * point.y + point.z * point.z) > 0.5) {
  //       filtered_cloud->points.push_back(point);
  //     }
  //   }

  //   sensor_msgs::msg::PointCloud2 output;
  //   pcl::toROSMsg(*filtered_cloud, output);
  //   output.header = msg->header;
  //   publisher_->publish(output);

  //   RCLCPP_INFO(this->get_logger(), "Filtered point cloud published with %zu points", filtered_cloud->points.size());
  // }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

  Eigen::Matrix3d rotation_180_x_, rotation_x_, rotation_y_, rotation_z_, rotation_matrix_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr pointcloud_subscription_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pointcloud_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudFilter>());
  rclcpp::shutdown();
  return 0;
}


