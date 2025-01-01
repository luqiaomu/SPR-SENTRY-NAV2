colcon build
cmds=(  #"ros2 launch pb_rm_simulation rm_simulation.launch.py "
	#"ros2 run teleop_twist_keyboard teleop_twist_keyboard"
	"ros2 run rviz2 rviz2"
	"ros2 launch sam_bot_description robot.launch.py"
	"ros2 launch livox_ros_driver2 msg_MID360_launch.py"
	"ros2 run lidar_preprocessing lidar_preprocessing"
	"ros2 launch imu_complementary_filter complementary_filter.launch.py"
	"ros2 launch linefit_ground_segmentation_ros segmentation.launch.py" 
	"ros2 launch fast_lio mapping_mid360.launch.py"
	"ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py"
	"ros2 launch pcl_laser_icp pcl_laser_icp.launch.py"
	)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done
