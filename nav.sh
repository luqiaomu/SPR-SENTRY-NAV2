colcon build
cmds=( #"ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom"
        "ros2 launch pcl_laser_icp pcl_laser_icp.launch.py"
 	"ros2 launch sam_bot_description robot.launch.py"
	"ros2 launch livox_ros_driver2 msg_MID360_launch.py"
	"ros2 launch linefit_ground_segmentation_ros segmentation.launch.py" 
	"ros2 launch fast_lio mapping_mid360.launch.py"
	
#	"ros2 run my_open3d_ros2_package open3d_ros2_rode"
	"ros2 launch imu_complementary_filter complementary_filter.launch.py"
	"ros2 run lidar_preprocessing lidar_preprocessing"
	"ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py"
	#"ros2 launch icp_registration icp.launch.py"
	#"ros2 launch pcl_laser_icp pcl_laser_icp.launch.py"
	"ros2 run my_open3d_ros2_package remove_multiplier" 
	"ros2 run controller rm_controller"
	"ros2 launch rm_navigation bringup_launch.py"
	#"ros2 run commander cmd_vel_subscriber"
	"ros2 run commander controller_serial"
	"ros2 launch commander commander.launch.py"
	
	
	#"ros2 run location_check location_check"
	)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done
