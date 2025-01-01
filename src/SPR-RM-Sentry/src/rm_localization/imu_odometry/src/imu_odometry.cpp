#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>

class ImuOdometry : public rclcpp::Node {
public:
    ImuOdometry() : Node("imu_odometry"), transform_(Eigen::Matrix4d::Identity()) {
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", 10, std::bind(&ImuOdometry::imuCallback, this, std::placeholders::_1));
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/imu/pose", 10);
        last_time_ = this->now(); // 初始化时间
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // 从IMU消息的header中获取时间戳
        rclcpp::Time current_time = msg->header.stamp;
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        // std::cout<<dt<<std::endl;
        // std::cout<<current_time.seconds()<<std::endl;

        if (dt < 0.001) {
            return;
        }

        // 角速度转换为弧度
        Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

        // 线性加速度
        Eigen::Vector3d accel(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z - 9.81);

        // 更新四元数
        Eigen::Quaterniond delta_q;
        delta_q.w() = 1;
        delta_q.x() = 0.5 * gyro.x() * dt;
        delta_q.y() = 0.5 * gyro.y() * dt;
        delta_q.z() = 0.5 * gyro.z() * dt;
        delta_q.normalize();

        Eigen::Matrix3d R = transform_.block<3, 3>(0, 0);  // 当前旋转矩阵
        Eigen::Matrix3d delta_R = delta_q.toRotationMatrix();  // 增量旋转矩阵

        // 更新旋转矩阵
        Eigen::Matrix3d new_R = R * delta_R;

        // 更新位置
        Eigen::Vector3d accel_world = new_R * accel;
        Eigen::Vector3d new_position = transform_.block<3, 1>(0, 3) + 0.5 * accel_world * dt * dt;

        // 更新4x4变换矩阵
        transform_.block<3, 3>(0, 0) = new_R;
        transform_.block<3, 1>(0, 3) = new_position;

        // 发布位姿数据
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = current_time;
        pose_msg.header.frame_id = "odom";
        pose_msg.pose.position.x = transform_(0, 3);
        pose_msg.pose.position.y = transform_(1, 3);
        pose_msg.pose.position.z = transform_(2, 3);
        Eigen::Quaterniond q(new_R);
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();

        pose_publisher_->publish(pose_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

    Eigen::Matrix4d transform_;
    rclcpp::Time last_time_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuOdometry>());
    rclcpp::shutdown();
    return 0;
}