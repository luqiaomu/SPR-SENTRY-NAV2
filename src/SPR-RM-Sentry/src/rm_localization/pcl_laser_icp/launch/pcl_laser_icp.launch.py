from launch import LaunchDescription           
from launch_ros.actions import Node     
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution       

def generate_launch_description():             
    init_x_launch_arg = DeclareLaunchArgument('x_init', default_value=TextSubstitution(text='1.0'))
    init_y_launch_arg = DeclareLaunchArgument('y_init', default_value=TextSubstitution(text='0.0'))
    init_yaw_launch_arg = DeclareLaunchArgument('yaw_init', default_value=TextSubstitution(text='0.0'))

    return LaunchDescription([                 
        init_x_launch_arg,
        init_y_launch_arg,
        init_yaw_launch_arg,
        
        Node(                                  
            package='pcl_laser_icp',          
            executable='pcl_laser_icp', 
            parameters=[{
                'x_init' : LaunchConfiguration('x_init'),
                'y_init' : LaunchConfiguration('y_init'),
                'yaw_init' : LaunchConfiguration('yaw_init'),
            }]
        ),
    ])
