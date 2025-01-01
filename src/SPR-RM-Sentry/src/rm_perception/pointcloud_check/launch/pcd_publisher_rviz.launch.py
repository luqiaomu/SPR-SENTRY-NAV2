from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 启动点云发布节点
        Node(
            package='pointcloud_check',  # 替换为您的包名
            executable='pcd_publisher',   # 替换为您的 Python 脚本的可执行名称
            name='pcd_publisher',
            output='screen'
        ),

        # 启动 Rviz 并配置其订阅 /pointcloud 主题
        ExecuteProcess(
            cmd=['rviz2', '-d', '/home/spr/Sentry_ws/src/SPR-RM-Sentry/src/rm_perception/pointcloud_check/config/pcl_config.rviz'],
            output='screen'
        ),
    ])

