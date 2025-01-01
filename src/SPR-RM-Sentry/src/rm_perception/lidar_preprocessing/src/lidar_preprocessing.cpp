


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



















// #include <rclcpp/rclcpp.hpp>
// #include <livox_ros_driver2/msg/custom_msg.hpp>
// #include <sensor_msgs/msg/imu.hpp>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/crop_box.h>
// #include <Eigen/Dense> // Eigen 库用于矩阵运算

// class PointCloudFilter : public rclcpp::Node {
// public:
//   PointCloudFilter() : Node("lidar_preprocessing") {
//     // 初始化旋转矩阵
//     initialize_rotation_matrices();

//     // 订阅 Livox 自定义点云消息
//     pointcloud_subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
//         "/livox/lidar", 10, std::bind(&PointCloudFilter::pointcloud_callback, this, std::placeholders::_1));

//     // 订阅 IMU 数据
//     imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
//         "/imu/data", 10, std::bind(&PointCloudFilter::imu_callback, this, std::placeholders::_1));

//     // 发布过滤后的点云
//     pointcloud_publisher_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("/filtered_pointcloud", 10);

//     // 发布翻转后的 IMU 数据
//     imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/livox/imu/filtered", 10);
//   }

// private:
//   void initialize_rotation_matrices() {
//     // 定义旋转角度
//     const double theta_x = -0.5* M_PI / 180.0; // 绕 X 轴倾斜-1.01
//     const double theta_y = 1.7* M_PI / 180.0; // 绕 Y 轴倾斜-1.78

//     // **绕 X 轴旋转 180° 的矩阵**
//     rotation_180_x_ << 1,  0,  0,
//                        0, -1,  0,
//                        0,  0, -1;

//     // **绕 X 轴倾斜的旋转矩阵**
//     rotation_x_ << 1, 0, 0,
//                    0, cos(theta_x), -sin(theta_x),
//                    0, sin(theta_x), cos(theta_x);

//     // **绕 Y 轴倾斜的旋转矩阵**
//     rotation_y_ << cos(theta_y), 0, sin(theta_y),
//                    0, 1, 0,
//                    -sin(theta_y), 0, cos(theta_y);

//     // 组合旋转矩阵
//     rotation_matrix_ = rotation_180_x_ * rotation_y_ * rotation_x_;
//   }

//   void pointcloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
//     pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);

//     if (msg->points.empty()) {
//       RCLCPP_WARN(this->get_logger(), "No points received, skipping processing.");
//       return;
//     }

//     // 从 Livox 自定义消息中提取点云数据并应用旋转矩阵
//     for (const auto &point : msg->points) {
//       pcl::PointXYZI pcl_point;
//       Eigen::Vector3d original_point(point.x, point.y, point.z);

//       // 应用旋转矩阵
//       Eigen::Vector3d rotated_point = rotation_matrix_ * original_point;

//       pcl_point.x = rotated_point.x();
//       pcl_point.y = rotated_point.y();
//       pcl_point.z = rotated_point.z();
//       pcl_point.intensity = point.reflectivity;

//       pcl_cloud->points.push_back(pcl_point);
//     }

//     // 使用 CropBox 过滤器
//     pcl::CropBox<pcl::PointXYZI> crop_box_filter;
//     crop_box_filter.setInputCloud(pcl_cloud);

//     Eigen::Vector4f min_point(-0.275, -0.30, -0.170, 1.0);
//     Eigen::Vector4f max_point(0.275, 0.30, 0.530, 1.0);
//     crop_box_filter.setMin(min_point);
//     crop_box_filter.setMax(max_point);
//     crop_box_filter.setNegative(true);

//     pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
//     crop_box_filter.filter(*filtered_cloud);

//     // 构造 Livox 自定义消息，保留原始字段
//     livox_ros_driver2::msg::CustomMsg output_msg = *msg;
//     output_msg.point_num = filtered_cloud->points.size();
//     output_msg.points.clear();

//     for (size_t i = 0; i < filtered_cloud->points.size(); ++i) {
//       livox_ros_driver2::msg::CustomPoint point;
//       point.x = filtered_cloud->points[i].x;
//       point.y = filtered_cloud->points[i].y;
//       point.z = filtered_cloud->points[i].z;
//       point.reflectivity = filtered_cloud->points[i].intensity;

//       // 保留原始的 offset_time、tag 和 line
//       point.offset_time = msg->points[i].offset_time;
//       point.tag = msg->points[i].tag;
//       point.line = msg->points[i].line;

//       output_msg.points.push_back(point);
//     }

//     pointcloud_publisher_->publish(output_msg);
//     RCLCPP_INFO(this->get_logger(), "Filtered and flipped point cloud published with %zu points", filtered_cloud->points.size());
//   }

//   void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
//     sensor_msgs::msg::Imu flipped_imu = *msg;

//     // 使用旋转矩阵翻转 IMU 数据
//     Eigen::Vector3d linear_acceleration(
//         msg->linear_acceleration.x,
//         msg->linear_acceleration.y,
//         msg->linear_acceleration.z);
//     linear_acceleration = rotation_matrix_ * linear_acceleration;

//     flipped_imu.linear_acceleration.x = linear_acceleration.x();
//     flipped_imu.linear_acceleration.y = linear_acceleration.y();
//     flipped_imu.linear_acceleration.z = linear_acceleration.z();

//     Eigen::Vector3d angular_velocity(
//         msg->angular_velocity.x,
//         msg->angular_velocity.y,
//         msg->angular_velocity.z);
//     angular_velocity = rotation_matrix_ * angular_velocity;

//     flipped_imu.angular_velocity.x = angular_velocity.x();
//     flipped_imu.angular_velocity.y = angular_velocity.y();
//     flipped_imu.angular_velocity.z = angular_velocity.z();

//     Eigen::Quaterniond orientation(
//         msg->orientation.w,
//         msg->orientation.x,
//         msg->orientation.y,
//         msg->orientation.z);
//     orientation = Eigen::Quaterniond(rotation_matrix_) * orientation;

//     flipped_imu.orientation.w = orientation.w();
//     flipped_imu.orientation.x = orientation.x();
//     flipped_imu.orientation.y = orientation.y();
//     flipped_imu.orientation.z = orientation.z();

//     imu_publisher_->publish(flipped_imu);
//     RCLCPP_INFO(this->get_logger(), "Flipped IMU data published.");
//   }

//   Eigen::Matrix3d rotation_180_x_, rotation_x_, rotation_y_, rotation_matrix_;
//   rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr pointcloud_subscription_;
//   rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pointcloud_publisher_;
//   rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
//   rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
// };

// int main(int argc, char *argv[]) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<PointCloudFilter>());
//   rclcpp::shutdown();
//   return 0;
// }





// #include <rclcpp/rclcpp.hpp>
// #include <livox_ros_driver2/msg/custom_msg.hpp>
// #include <sensor_msgs/msg/imu.hpp>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/crop_box.h>
// #include <Eigen/Dense> // Eigen 库用于矩阵运算

// class PointCloudFilter : public rclcpp::Node {
// public:
//   PointCloudFilter() : Node("lidar_preprocessing") {
//     // 初始化旋转矩阵
//     initialize_rotation_matrices();

//     // 订阅 Livox 自定义点云消息
//     pointcloud_subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
//         "/livox/lidar", 10, std::bind(&PointCloudFilter::pointcloud_callback, this, std::placeholders::_1));

//     // 订阅 IMU 数据
//     imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
//         "/imu/data", 10, std::bind(&PointCloudFilter::imu_callback, this, std::placeholders::_1));

//     // 发布过滤后的点云
//     pointcloud_publisher_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("/filtered_pointcloud", 10);

//     // 发布翻转后的 IMU 数据
//     imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/livox/imu/filtered", 10);
//   }

// private:
//   void initialize_rotation_matrices() {
//     // 定义旋转角度
//     const double theta_x = 1.0 * M_PI / 180.0; // 绕 X 轴倾斜-1.01 
//     const double theta_y = -1.2* M_PI / 180.0; // 绕 Y 轴倾斜-1.78 

//     // **绕 X 轴旋转 180° 的矩阵**
//     rotation_180_x_ << 1,  0,  0,
//                        0, -1,  0,
//                        0,  0, -1;

//     // **绕 X 轴倾斜的旋转矩阵**
//     rotation_x_ << 1, 0, 0,
//                    0, cos(theta_x), -sin(theta_x),
//                    0, sin(theta_x), cos(theta_x);

//     // **绕 Y 轴倾斜的旋转矩阵**
//     rotation_y_ << cos(theta_y), 0, sin(theta_y),
//                    0, 1, 0,
//                    -sin(theta_y), 0, cos(theta_y);

//     // 组合旋转矩阵
//     rotation_matrix_ = rotation_180_x_ * rotation_y_ * rotation_x_;
//   }

//   void pointcloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
//     pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);

//     if (msg->points.empty()) {
//       RCLCPP_WARN(this->get_logger(), "No points received, skipping processing.");
//       return;
//     }

//     // 从 Livox 自定义消息中提取点云数据并应用旋转矩阵
//     for (const auto &point : msg->points) {
//       pcl::PointXYZI pcl_point;
//       Eigen::Vector3d original_point(point.x, point.y, point.z);

//       // 应用旋转矩阵
//       Eigen::Vector3d rotated_point = rotation_matrix_ * original_point;

//       pcl_point.x = rotated_point.x();
//       pcl_point.y = rotated_point.y();
//       pcl_point.z = rotated_point.z();
//       pcl_point.intensity = point.reflectivity;

//       pcl_cloud->points.push_back(pcl_point);
//     }

//     // 使用 CropBox 过滤器
//     pcl::CropBox<pcl::PointXYZI> crop_box_filter;
//     crop_box_filter.setInputCloud(pcl_cloud);

//     Eigen::Vector4f min_point(-0.275, -0.30, -0.170, 1.0);
//     Eigen::Vector4f max_point(0.275, 0.30, 0.530, 1.0);
//     crop_box_filter.setMin(min_point);
//     crop_box_filter.setMax(max_point);
//     crop_box_filter.setNegative(true);

//     pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
//     crop_box_filter.filter(*filtered_cloud);

//     // 构造 Livox 自定义消息
//     livox_ros_driver2::msg::CustomMsg output_msg;
//     output_msg.header = msg->header;
//     output_msg.timebase = msg->timebase;
//     output_msg.lidar_id = msg->lidar_id;
//     output_msg.rsvd = msg->rsvd;
//     output_msg.point_num = filtered_cloud->points.size();

//     for (size_t i = 0; i < filtered_cloud->points.size(); ++i) {
//       livox_ros_driver2::msg::CustomPoint point;
//       point.x = filtered_cloud->points[i].x;
//       point.y = filtered_cloud->points[i].y;
//       point.z = filtered_cloud->points[i].z;
//       point.reflectivity = filtered_cloud->points[i].intensity;

//       point.offset_time = msg->points[i].offset_time;
//       point.tag = msg->points[i].tag;
//       point.line = msg->points[i].line;

//       output_msg.points.push_back(point);
//     }

//     pointcloud_publisher_->publish(output_msg);
//     RCLCPP_INFO(this->get_logger(), "Filtered and flipped point cloud published with %zu points", filtered_cloud->points.size());
//   }

//   void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
//     sensor_msgs::msg::Imu flipped_imu = *msg;

//     // 使用旋转矩阵翻转 IMU 数据
//     Eigen::Vector3d linear_acceleration(
//         msg->linear_acceleration.x,
//         msg->linear_acceleration.y,
//         msg->linear_acceleration.z);
//     linear_acceleration = rotation_matrix_ * linear_acceleration;

//     flipped_imu.linear_acceleration.x = linear_acceleration.x();
//     flipped_imu.linear_acceleration.y = linear_acceleration.y();
//     flipped_imu.linear_acceleration.z = linear_acceleration.z();

//     Eigen::Vector3d angular_velocity(
//         msg->angular_velocity.x,
//         msg->angular_velocity.y,
//         msg->angular_velocity.z);
//     angular_velocity = rotation_matrix_ * angular_velocity;

//     flipped_imu.angular_velocity.x = angular_velocity.x();
//     flipped_imu.angular_velocity.y = angular_velocity.y();
//     flipped_imu.angular_velocity.z = angular_velocity.z();

//     Eigen::Quaterniond orientation(
//         msg->orientation.w,
//         msg->orientation.x,
//         msg->orientation.y,
//         msg->orientation.z);
//     orientation = Eigen::Quaterniond(rotation_matrix_) * orientation;

//     flipped_imu.orientation.w = orientation.w();
//     flipped_imu.orientation.x = orientation.x();
//     flipped_imu.orientation.y = orientation.y();
//     flipped_imu.orientation.z = orientation.z();

//     imu_publisher_->publish(flipped_imu);
//     RCLCPP_INFO(this->get_logger(), "Flipped IMU data published.");
//   }

//   Eigen::Matrix3d rotation_180_x_, rotation_x_, rotation_y_, rotation_matrix_;
//   rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr pointcloud_subscription_;
//   rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pointcloud_publisher_;
//   rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
//   rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
// };

// int main(int argc, char *argv[]) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<PointCloudFilter>());
//   rclcpp::shutdown();
//   return 0;
// }






// #include <rclcpp/rclcpp.hpp>
// #include <livox_ros_driver2/msg/custom_msg.hpp>
// #include <sensor_msgs/msg/imu.hpp>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/crop_box.h>
// #include <Eigen/Dense> // Eigen 库用于矩阵运算

// class PointCloudFilter : public rclcpp::Node {
// public:
//   PointCloudFilter() : Node("lidar_preprocessing") {
//     // 初始化旋转矩阵，表示绕 X 轴翻转 180 度
//     rotation_matrix_ << 1,  0,  0,
//                         0, -1,  0,
//                         0,  0, -1;

//     // 订阅 Livox 自定义点云消息
//     pointcloud_subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
//         "/livox/lidar", 10, std::bind(&PointCloudFilter::pointcloud_callback, this, std::placeholders::_1));

//     // 订阅 IMU 数据
//     imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
//         "/imu/data", 10, std::bind(&PointCloudFilter::imu_callback, this, std::placeholders::_1));

//     // 发布过滤后的点云
//     pointcloud_publisher_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("/filtered_pointcloud", 10);

//     // 发布翻转后的 IMU 数据
//     imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/livox/imu/filtered", 10);
//   }

// private:
//   void pointcloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
//     pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
//     RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points", msg->points.size());
//     if (msg->points.empty()) {
//       RCLCPP_WARN(this->get_logger(), "No points received, skipping processing.");
//       return;
//     }

//     // 从 Livox 自定义消息中提取点云数据并翻转 YZ 轴
//     for (const auto &point : msg->points) {
//       pcl::PointXYZI pcl_point;
//       pcl_point.x = point.x;
//       pcl_point.y = -point.y; // 翻转 Y 轴
//       pcl_point.z = -point.z; // 翻转 Z 轴
//       pcl_point.intensity = point.reflectivity;
//       pcl_cloud->points.push_back(pcl_point);
//     }

//     // 使用 CropBox 过滤器
//     pcl::CropBox<pcl::PointXYZI> crop_box_filter;
//     crop_box_filter.setInputCloud(pcl_cloud);

//     Eigen::Vector4f min_point(-0.275, -0.30, -0.170, 1.0);
//     Eigen::Vector4f max_point(0.275, 0.30, 0.530, 1.0);
//     crop_box_filter.setMin(min_point);
//     crop_box_filter.setMax(max_point);
//     crop_box_filter.setNegative(true);

//     pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
//     crop_box_filter.filter(*filtered_cloud);

//     // 构造 Livox 自定义消息
//     livox_ros_driver2::msg::CustomMsg output_msg;
//     output_msg.header = msg->header;
//     output_msg.timebase = msg->timebase;
//     output_msg.lidar_id = msg->lidar_id;
//     output_msg.rsvd = msg->rsvd;
//     output_msg.point_num = filtered_cloud->points.size();

//     for (size_t i = 0; i < filtered_cloud->points.size(); ++i) {
//       livox_ros_driver2::msg::CustomPoint point;
//       point.x = filtered_cloud->points[i].x;
//       point.y = filtered_cloud->points[i].y;
//       point.z = filtered_cloud->points[i].z;
//       point.reflectivity = filtered_cloud->points[i].intensity;

//       point.offset_time = msg->points[i].offset_time;
//       point.tag = msg->points[i].tag;
//       point.line = msg->points[i].line;

//       output_msg.points.push_back(point);
//     }

//     pointcloud_publisher_->publish(output_msg);
//     RCLCPP_INFO(this->get_logger(), "Filtered and flipped point cloud published with %zu points", filtered_cloud->points.size());
//   }

//   void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
//     sensor_msgs::msg::Imu flipped_imu = *msg;

//     // 使用旋转矩阵翻转 IMU 数据
//     Eigen::Vector3d linear_acceleration(
//         msg->linear_acceleration.x,
//         msg->linear_acceleration.y,
//         msg->linear_acceleration.z);
//     linear_acceleration = rotation_matrix_ * linear_acceleration;

//     flipped_imu.linear_acceleration.x = linear_acceleration.x();
//     flipped_imu.linear_acceleration.y = linear_acceleration.y();
//     flipped_imu.linear_acceleration.z = linear_acceleration.z();

//     Eigen::Vector3d angular_velocity(
//         msg->angular_velocity.x,
//         msg->angular_velocity.y,
//         msg->angular_velocity.z);
//     angular_velocity = rotation_matrix_ * angular_velocity;

//     flipped_imu.angular_velocity.x = angular_velocity.x();
//     flipped_imu.angular_velocity.y = angular_velocity.y();
//     flipped_imu.angular_velocity.z = angular_velocity.z();

//     Eigen::Quaterniond orientation(
//         msg->orientation.w,
//         msg->orientation.x,
//         msg->orientation.y,
//         msg->orientation.z);
//     orientation = Eigen::Quaterniond(rotation_matrix_) * orientation;

//     flipped_imu.orientation.w = orientation.w();
//     flipped_imu.orientation.x = orientation.x();
//     flipped_imu.orientation.y = orientation.y();
//     flipped_imu.orientation.z = orientation.z();

//     imu_publisher_->publish(flipped_imu);
//     RCLCPP_INFO(this->get_logger(), "Flipped IMU data published.");
//   }

//   Eigen::Matrix3d rotation_matrix_;
//   rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr pointcloud_subscription_;
//   rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pointcloud_publisher_;
//   rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
//   rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
// };

// int main(int argc, char *argv[]) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<PointCloudFilter>());
//   rclcpp::shutdown();
//   return 0;
// }


 
















// #include <rclcpp/rclcpp.hpp>
// #include <livox_ros_driver2/msg/custom_msg.hpp>
// #include <sensor_msgs/msg/imu.hpp>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/crop_box.h>

// class PointCloudFilter : public rclcpp::Node {
// public:
//   PointCloudFilter() : Node("lidar_preprocessing") {
//     // 订阅 Livox 自定义点云消息
//     pointcloud_subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
//         "/livox/lidar", 10, std::bind(&PointCloudFilter::pointcloud_callback, this, std::placeholders::_1));

//     // 订阅 IMU 数据
//     imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
//         "/livox/imu", 10, std::bind(&PointCloudFilter::imu_callback, this, std::placeholders::_1));

//     // 发布过滤后的点云
//     pointcloud_publisher_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("/filtered_pointcloud", 10);

//     // 发布翻转后的 IMU 数据
//     imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/livox/imu/filtered", 10);
//   }

// private:
//   void pointcloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
//     pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
//     RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points", msg->points.size());
//     if (msg->points.empty()) {
//       RCLCPP_WARN(this->get_logger(), "No points received, skipping processing.");
//       return;
//     }

//     // 从 Livox 自定义消息中提取点云数据并翻转 YZ 轴
//     for (const auto &point : msg->points) {
//       pcl::PointXYZI pcl_point;
//       pcl_point.x = point.x;
//       pcl_point.y = -point.y; // 翻转 Y 轴
//       pcl_point.z = -point.z; // 翻转 Z 轴
//       pcl_point.intensity = point.reflectivity;
//       pcl_cloud->points.push_back(pcl_point);
//     }

//     // 使用 CropBox 过滤器
//     pcl::CropBox<pcl::PointXYZI> crop_box_filter;
//     crop_box_filter.setInputCloud(pcl_cloud);

//     Eigen::Vector4f min_point(-0.275, -0.30, -0.170, 1.0);
//     Eigen::Vector4f max_point(0.275, 0.30, 0.530, 1.0);
//     crop_box_filter.setMin(min_point);
//     crop_box_filter.setMax(max_point);
//     crop_box_filter.setNegative(true);

//     pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
//     crop_box_filter.filter(*filtered_cloud);

//     // 构造 Livox 自定义消息
//     livox_ros_driver2::msg::CustomMsg output_msg;
//     output_msg.header = msg->header;
//     output_msg.timebase = msg->timebase;
//     output_msg.lidar_id = msg->lidar_id;
//     output_msg.rsvd = msg->rsvd;
//     output_msg.point_num = filtered_cloud->points.size();

//     for (size_t i = 0; i < filtered_cloud->points.size(); ++i) {
//       livox_ros_driver2::msg::CustomPoint point;
//       point.x = filtered_cloud->points[i].x;
//       point.y = filtered_cloud->points[i].y;
//       point.z = filtered_cloud->points[i].z;
//       point.reflectivity = filtered_cloud->points[i].intensity;

//       point.offset_time = msg->points[i].offset_time;
//       point.tag = msg->points[i].tag;
//       point.line = msg->points[i].line;

//       output_msg.points.push_back(point);
//     }

//     pointcloud_publisher_->publish(output_msg);
//     RCLCPP_INFO(this->get_logger(), "Filtered and flipped point cloud published with %zu points", filtered_cloud->points.size());
//   }

//   void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
//     sensor_msgs::msg::Imu flipped_imu = *msg;

//     // 翻转 IMU 数据的 Y 和 Z 方向
//     flipped_imu.linear_acceleration.y = -msg->linear_acceleration.y;
//     flipped_imu.linear_acceleration.z = -msg->linear_acceleration.z;
//     flipped_imu.angular_velocity.y = -msg->angular_velocity.y;
//     flipped_imu.angular_velocity.z = -msg->angular_velocity.z;
//     flipped_imu.orientation.y = -msg->orientation.y;
//     flipped_imu.orientation.z = -msg->orientation.z;

//     imu_publisher_->publish(flipped_imu);
//     RCLCPP_INFO(this->get_logger(), "Flipped IMU data published.");
//   }

//   // 点云订阅者和发布者
//   rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr pointcloud_subscription_;
//   rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pointcloud_publisher_;

//   // IMU订阅者和发布者
//   rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
//   rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
// };

// int main(int argc, char *argv[]) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<PointCloudFilter>());
//   rclcpp::shutdown();
//   return 0;
// }





              
// #include <rclcpp/rclcpp.hpp>
// #include <livox_ros_driver2/msg/custom_msg.hpp>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/crop_box.h>

// class PointCloudFilter : public rclcpp::Node
// {
// public:
//   PointCloudFilter() : Node("lidar_preprocessing")
//   {
//     // 订阅 Livox 自定义点云消息
//     subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
//       "/livox/lidar", 10, std::bind(&PointCloudFilter::topic_callback, this, std::placeholders::_1));
    
//     // 发布过滤后的点云
//     publisher_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("/filtered_pointcloud", 10);
//   }

// private:
//   void topic_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
//   {
//     // 创建 PCL 点云
//     pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
//     RCLCPP_INFO(this->get_logger(), "Received point cloud with %zu points", msg->points.size());
//     if (msg->points.empty()) {
//       RCLCPP_WARN(this->get_logger(), "No points received, skipping processing.");
//       return;
//     }

//     // 从 Livox 自定义消息中提取点云数据
//     for (const auto &point : msg->points)
//     {
//       pcl::PointXYZI pcl_point;
//       pcl_point.x = point.x;
//       pcl_point.y = point.y;
//       pcl_point.z = point.z;
//       pcl_point.intensity = point.reflectivity;
//       pcl_cloud->points.push_back(pcl_point);
//     }

//     // 使用 CropBox 过滤器
//     pcl::CropBox<pcl::PointXYZI> crop_box_filter;
//     crop_box_filter.setInputCloud(pcl_cloud);

//     // 设置长方体的范围 (最小点和最大点)
//     Eigen::Vector4f min_point(-0.275, -0.30, -0.170, 1.0); // 长方体的最小点 (x, y, z)
//     Eigen::Vector4f max_point(0.275, 0.30, 0.530, 1.0);   // 长方体的最大点 (x, y, z)

//     crop_box_filter.setMin(min_point);
//     crop_box_filter.setMax(max_point);

//     // 设置为负值以保留范围外的点
//     crop_box_filter.setNegative(true); // 只保留长方体外的点
//     pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
//     crop_box_filter.filter(*filtered_cloud);

//     // 构造 Livox 自定义消息
//     livox_ros_driver2::msg::CustomMsg output_msg;
//     output_msg.header = msg->header;
//     output_msg.timebase = msg->timebase; // 保留原始时间基准
//     output_msg.lidar_id = msg->lidar_id; // 保留原始 LiDAR ID
//     output_msg.rsvd = msg->rsvd;         // 保留原始保留字段
//     output_msg.point_num = filtered_cloud->points.size(); // 更新点数量

//     // 使用原始索引对其他字段进行映射
//     for (size_t i = 0; i < filtered_cloud->points.size(); ++i)
//     {
//       livox_ros_driver2::msg::CustomPoint point;
//       point.x = filtered_cloud->points[i].x;
//       point.y = filtered_cloud->points[i].y;
//       point.z = filtered_cloud->points[i].z;
//       point.reflectivity = filtered_cloud->points[i].intensity;
      
//       // 使用原始消息的字段值
//       point.offset_time = msg->points[i].offset_time;
//       point.tag = msg->points[i].tag;
//       point.line = msg->points[i].line;

//       output_msg.points.push_back(point);
//     }

//     // 发布过滤后的点云
//     publisher_->publish(output_msg);
//     RCLCPP_INFO(this->get_logger(), "Filtered point cloud published with %zu points", filtered_cloud->points.size());
//   }

//   rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
//   rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr publisher_;
// };

// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<PointCloudFilter>());
//   rclcpp::shutdown();
//   return 0;
// }

// #include <rclcpp/rclcpp.hpp>
// #include <livox_ros_driver2/msg/custom_msg.hpp>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/crop_box.h>

// class PointCloudFilter : public rclcpp::Node
// {
// public:
//   PointCloudFilter() : Node("lidar_preprocessing")
//   {
//     // 订阅 Livox 自定义点云消息
//     subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
//       "/livox/lidar", 10, std::bind(&PointCloudFilter::topic_callback, this, std::placeholders::_1));
    
//     // 发布过滤后的点云
//     publisher_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("/filtered_pointcloud", 10);
//   }

// private:
//   void topic_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
//   {
//     // 创建 PCL 点云
//     pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);

//     // 从 Livox 自定义消息中提取点云数据
//     for (const auto &point : msg->points)
//     {
//       pcl::PointXYZI pcl_point;
//       pcl_point.x = point.x;
//       pcl_point.y = point.y;
//       pcl_point.z = point.z;
//       pcl_point.intensity = point.reflectivity;
//       pcl_cloud->points.push_back(pcl_point);
//     }

//     // 使用 CropBox 过滤器
//     pcl::CropBox<pcl::PointXYZI> crop_box_filter;
//     crop_box_filter.setInputCloud(pcl_cloud);


//     // Eigen::Vector4f min_point(0,0,0); // 长方体的最小点 (x, y, z)
//     // Eigen::Vector4f max_point(0,0,0);   // 长方体的最大点 (x, y, z)

//     设置长方体的范围 (最小点和最大点)
//     Eigen::Vector4f min_point(-0.275, -0.30, -0.170, 1.0); // 长方体的最小点 (x, y, z)
//     Eigen::Vector4f max_point(0.275, 0.30, 0.530, 1.0);   // 长方体的最大点 (x, y, z)

//     crop_box_filter.setMin(min_point);
//     crop_box_filter.setMax(max_point);

//   //  设置为负值以保留范围外的点
//     crop_box_filter.setNegative(true); // 只保留长方体外的点
//     pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
//     crop_box_filter.filter(*filtered_cloud);

//     // 构造 Livox 自定义消息
//     livox_ros_driver2::msg::CustomMsg output_msg;
//     output_msg.header = msg->header;

//     for (const auto &pcl_point : filtered_cloud->points)
//     {
//       livox_ros_driver2::msg::CustomPoint point;
//       point.x = pcl_point.x;
//       point.y = pcl_point.y;
//       point.z = pcl_point.z;
//       point.reflectivity = pcl_point.intensity;
//       output_msg.points.push_back(point);
//     }

//     // 发布过滤后的点云
//     publisher_->publish(output_msg);
//     RCLCPP_INFO(this->get_logger(), "Filtered point cloud published with %zu points", filtered_cloud->points.size());
//   }

//   rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
//   rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr publisher_;
// };

// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<PointCloudFilter>());
//   rclcpp::shutdown();
//   return 0;
// }















// // #include <rclcpp/rclcpp.hpp>
// // #include <sensor_msgs/msg/point_cloud2.hpp>
// // #include <sensor_msgs/msg/imu.hpp>
// // #include <pcl_conversions/pcl_conversions.h>
// // #include <pcl/point_cloud.h>
// // #include <pcl/point_types.h>
// // #include <pcl/filters/crop_box.h>
// // #include <pcl/filters/voxel_grid.h>
// // #include <pcl/features/normal_3d.h>
// // #include <pcl/ModelCoefficients.h>
// // #include <pcl/segmentation/sac_segmentation.h>
// // #include <pcl/filters/extract_indices.h>

// // class PointCloudFilter : public rclcpp::Node
// // {
// // public:
// //   PointCloudFilter() : Node("lidar_preprocessing")
// //   {
// //     // 订阅 Livox 激光雷达点云数据的主题
// //     cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
// //       "/livox/lidar/pointcloud", 10, std::bind(&PointCloudFilter::cloud_callback, this, std::placeholders::_1));

// //     // 订阅 IMU 数据的主题
// //     imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
// //       "/livox/imu", 10, std::bind(&PointCloudFilter::imu_callback, this, std::placeholders::_1));

// //     // 发布过滤后的点云数据
// //     cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_pointcloud", 10);

// //     // 发布翻转后的 IMU 数据
// //     imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/livox_filter_imu", 10);
// //   }

// // private:
// //   // 点云数据的回调函数
// //   void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
// //   {
// //     // 1. 将 ROS 点云消息转换为 PCL 点云格式
// //     pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
// //     pcl::fromROSMsg(*msg, *pcl_cloud);

// //     // 2. 使用 CropBox 过滤器裁剪点云，只保留感兴趣区域以外的点
// //     pcl::CropBox<pcl::PointXYZI> crop_box_filter;
// //     crop_box_filter.setInputCloud(pcl_cloud);

// //     // 设置裁剪范围的最小点和最大点（单位为米）
// //     Eigen::Vector4f min_point(-0.275, -0.30, -0.170, 1.0); // 指定长方体裁剪的最小点坐标 (x, y, z)
// //     Eigen::Vector4f max_point(0.275, 0.30, 0.530, 1.0);    // 指定长方体裁剪的最大点坐标 (x, y, z)
// //     crop_box_filter.setMin(min_point);
// //     crop_box_filter.setMax(max_point);

// //     // 设置为负值以保留长方体区域外的点
// //     crop_box_filter.setNegative(true); // true 保留在该范围外的点
// //     pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZI>);
// //     crop_box_filter.filter(*cropped_cloud);

// //     // 3. 使用体素滤波器对裁剪后的点云进行下采样，减少点云数据量
// //     pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
// //     pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
// //     voxel_filter.setInputCloud(cropped_cloud);
// //     voxel_filter.setLeafSize(0.01f, 0.02f, 0.05f); // 设置体素的大小为0.1m
// //     voxel_filter.filter(*voxel_filtered_cloud);

// //     // 4. 计算点云法向量，以进行进一步的点云处理或分割
// //     pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimation;
// //     normal_estimation.setInputCloud(voxel_filtered_cloud);
// //     pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
// //     normal_estimation.setSearchMethod(tree);
// //     pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
// //     normal_estimation.setRadiusSearch(0.3); // 设置半径搜索范围
// //     normal_estimation.compute(*normals);

// //     // 5. 使用 RANSAC 算法分割平面，将平面内的点剔除
// //     pcl::SACSegmentation<pcl::PointXYZI> seg;
// //     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
// //     pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
// //     seg.setOptimizeCoefficients(true);
// //     seg.setModelType(pcl::SACMODEL_PLANE);
// //     seg.setMethodType(pcl::SAC_RANSAC);
// //     seg.setInputCloud(voxel_filtered_cloud);
// //     seg.setDistanceThreshold(0.015); // 设置平面距离阈值
// //     seg.segment(*inliers, *coefficients);

// //     // 6. 提取平面以外的点并生成新的点云
// //     pcl::ExtractIndices<pcl::PointXYZI> extract;
// //     pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
// //     extract.setInputCloud(voxel_filtered_cloud);
// //     extract.setIndices(inliers);
// //     extract.setNegative(true); // true 提取平面外的点
// //     extract.filter(*filtered_cloud);

// //     // 7. 翻转点云数据的 y 和 z 坐标，实现 180 度翻转
// //     for (auto &point : filtered_cloud->points)
// //     {
// //       point.y = -point.y; // 反转 y 坐标
// //       point.z = -point.z; // 反转 z 坐标
// //     }

// //     // 8. 将过滤后的点云从 PCL 格式转换回 ROS 消息并发布
// //     sensor_msgs::msg::PointCloud2 output;
// //     pcl::toROSMsg(*filtered_cloud, output);
// //     output.header = msg->header;
// //     cloud_publisher_->publish(output);

    
// //     RCLCPP_INFO(this->get_logger(), "Filtered point cloud published with %zu points", filtered_cloud->points.size());
  

// //   }

// //   // IMU 数据的回调函数，用于翻转 IMU 数据的 y 和 z 方向
// //   void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
// //   {
// //     // 创建一个新的 IMU 消息并复制原始数据
// //     auto flipped_imu = *msg;

// //     // 翻转 IMU 的线性加速度的 y 和 z 轴
// //     flipped_imu.linear_acceleration.y = -msg->linear_acceleration.y;
// //     flipped_imu.linear_acceleration.z = -msg->linear_acceleration.z;

// //     // 翻转 IMU 的角速度的 y 和 z 轴
// //     flipped_imu.angular_velocity.y = -msg->angular_velocity.y;
// //     flipped_imu.angular_velocity.z = -msg->angular_velocity.z;

// //     // 翻转方向四元数的 y 和 z 分量（相当于将方向翻转 180 度）
// //     flipped_imu.orientation.y = -msg->orientation.y;
// //     flipped_imu.orientation.z = -msg->orientation.z;

// //     // 发布翻转后的 IMU 数据
// //     imu_publisher_->publish(flipped_imu);
// //   }

// //   // 点云数据的订阅者
// //   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;

// //   // IMU 数据的订阅者
// //   rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

// //   // 过滤后点云数据的发布者
// //   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;

// //   // 翻转后的 IMU 数据的发布者
// //   rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
// // };

// // int main(int argc, char *argv[])
// // {
// //   // 初始化 ROS 2 节点
// //   rclcpp::init(argc, argv);

// //   // 创建节点实例并执行回调
// //   rclcpp::spin(std::make_shared<PointCloudFilter>());

// //   // 关闭 ROS 2 节点
// //   rclcpp::shutdown();
// //   return 0;
// // }














// // #include <rclcpp/rclcpp.hpp>
// // #include <sensor_msgs/msg/point_cloud2.hpp>
// // #include <pcl_conversions/pcl_conversions.h>
// // #include <pcl/point_cloud.h>
// // #include <pcl/point_types.h>
// // #include <pcl/filters/crop_box.h>
// // #include <pcl/filters/voxel_grid.h>
// // #include <pcl/features/normal_3d.h>
// // #include <pcl/ModelCoefficients.h>
// // #include <pcl/segmentation/sac_segmentation.h>
// // #include <pcl/filters/extract_indices.h>

// // class PointCloudFilter : public rclcpp::Node
// // {
// // public:
// //   PointCloudFilter() : Node("lidar_preprocessing")
// //   {
// //     subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
// //       "/livox/lidar/pointcloud", 10, std::bind(&PointCloudFilter::topic_callback, this, std::placeholders::_1));
    
// //     publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_pointcloud", 10);
// //   }

// // private:
// //   void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
// //   {
// //     // 转换ROS点云消息到PCL点云
// //     pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
// //     pcl::fromROSMsg(*msg, *pcl_cloud);

// //     // 1. 使用 CropBox 过滤器
// //     pcl::CropBox<pcl::PointXYZI> crop_box_filter;
// //     crop_box_filter.setInputCloud(pcl_cloud);

// //     // 设置长方体的范围 (最小点和最大点)
// //     Eigen::Vector4f min_point(-0.275, -0.30, -0.170, 1.0); // 长方体的最小点 (x, y, z)
// //     Eigen::Vector4f max_point(0.275, 0.30, 0.530, 1.0);   // 长方体的最大点 (x, y, z)

// //     crop_box_filter.setMin(min_point);
// //     crop_box_filter.setMax(max_point);

// //     // 设置为负值以保留范围外的点
// //     crop_box_filter.setNegative(true); // 只保留长方体外的点
// //     pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZI>);
// //     crop_box_filter.filter(*cropped_cloud);

// //     // 2. 体素网格滤波
// //     pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
// //     pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
// //     voxel_filter.setInputCloud(cropped_cloud);
// //     voxel_filter.setLeafSize(0.01f, 0.01f, 0.03f); // 设置体素大小
// //     voxel_filter.filter(*voxel_filtered_cloud);

// //     // 3. 法向量计算
// //     pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimation;
// //     normal_estimation.setInputCloud(voxel_filtered_cloud);
// //     pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
// //     normal_estimation.setSearchMethod(tree);
// //     pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
// //     normal_estimation.setRadiusSearch(0.3); // 半径搜索
// //     normal_estimation.compute(*normals);

// //     // 4. 使用 RANSAC 进行平面分离
// //     pcl::SACSegmentation<pcl::PointXYZI> seg;
// //     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
// //     pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    
// //     seg.setOptimizeCoefficients(true);
// //     seg.setModelType(pcl::SACMODEL_PLANE);
// //     seg.setMethodType(pcl::SAC_RANSAC);
// //     seg.setInputCloud(voxel_filtered_cloud);
// //     seg.setDistanceThreshold(0.015); // 平面模型的距离阈值0.01
// //     seg.segment(*inliers, *coefficients);

// //     // 提取平面外的点
// //     pcl::ExtractIndices<pcl::PointXYZI> extract;
// //     pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
// //     extract.setInputCloud(voxel_filtered_cloud);
// //     extract.setIndices(inliers);
// //     extract.setNegative(true); // 提取平面外的点
// //     extract.filter(*filtered_cloud);

// //   for (auto &point : filtered_cloud->points)
// //   {
// //     point.y = -point.y;
// //     point.z = -point.z;
// //   }




// //     // 转换PCL点云到ROS点云消息
// //     sensor_msgs::msg::PointCloud2 output;
// //     pcl::toROSMsg(*filtered_cloud, output);
// //     output.header = msg->header;

// //     // 发布过滤后的点云
// //     publisher_->publish(output);
// //     RCLCPP_INFO(this->get_logger(), "Filtered point cloud published with %zu points", filtered_cloud->points.size());
// //   }

// //   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
// //   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
// // };

// // int main(int argc, char *argv[])
// // {
// //   rclcpp::init(argc, argv);
// //   rclcpp::spin(std::make_shared<PointCloudFilter>());
// //   rclcpp::shutdown();
// //   return 0;
// // }




















// // // // // #include <rclcpp/rclcpp.hpp>
// // // // // #include <sensor_msgs/msg/point_cloud2.hpp>
// // // // // #include <pcl_conversions/pcl_conversions.h>
// // // // // #include <pcl/point_cloud.h>
// // // // // #include <pcl/point_types.h>
// // // // // #include <pcl/filters/passthrough.h>
// // // // // #include <pcl/filters/radius_outlier_removal.h>

// // // // // class PointCloudFilter : public rclcpp::Node
// // // // // {
// // // // // public:
// // // // //   PointCloudFilter() : Node("lidar_preprocessing")
// // // // //   {
// // // // //     subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
// // // // //       "/livox/lidar/pointcloud", 10, std::bind(&PointCloudFilter::topic_callback, this, std::placeholders::_1));
    
// // // // //     publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_pointcloud", 10);
// // // // //   }

// // // // // private:
// // // // //   void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
// // // // //   {
// // // // //     pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
// // // // //     pcl::fromROSMsg(*msg, *pcl_cloud);

// // // // //     pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
// // // // //     for (const auto& point : pcl_cloud->points) {
// // // // //       if (sqrt(point.x * point.x + point.y * point.y + point.z * point.z) > 0.5) {
// // // // //         filtered_cloud->points.push_back(point);
// // // // //       }
// // // // //     }

// // // // //     sensor_msgs::msg::PointCloud2 output;
// // // // //     pcl::toROSMsg(*filtered_cloud, output);
// // // // //     output.header = msg->header;
// // // // //     publisher_->publish(output);

// // // // //     RCLCPP_INFO(this->get_logger(), "Filtered point cloud published with %zu points", filtered_cloud->points.size());
// // // // //   }

// // // // //   rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
// // // // //   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
// // // // // };

// // // // // int main(int argc, char *argv[])
// // // // // {
// // // // //   rclcpp::init(argc, argv);
// // // // //   rclcpp::spin(std::make_shared<PointCloudFilter>());
// // // // //   rclcpp::shutdown();
// // // // //   return 0;
// // // // // }
