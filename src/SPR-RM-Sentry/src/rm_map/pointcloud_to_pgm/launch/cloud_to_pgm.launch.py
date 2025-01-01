import os

from ament_index_python.packages import get_package_share_directory  # 查询功能包路径的方法

from launch import LaunchDescription   # launch文件的描述类
from launch_ros.actions import Node    # 节点启动的描述类


def generate_launch_description():     # 自动生成launch文件的函数
   config='/home/spr/Sentry_ws/src/SPR-RM-Sentry/src/rm_map/pointcloud_to_pgm/config/param_ros.yaml'
   # config = os.path.join(              # 找到参数文件的完整路径
   #    get_package_share_directory('pointcloud_to_pgm'),
   #    'config',
   #    'param_ros.yaml'
   #    )

   return LaunchDescription([          # 返回launch文件的描述信息
      Node(                            # 配置一个节点的启动
         package='pointcloud_to_pgm',          # 节点所在的功能包
         executable='cloud_to_pgm',  # 节点的可执行文件名
         parameters=[config]           # 加载参数文件
      )
   ])