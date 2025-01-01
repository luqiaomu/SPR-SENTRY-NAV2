colcon build
cmds=( "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
       "rviz2"
        #"ros2 launch livox_ros_driver2 rviz_MID360_launch.py"
        "ros2 launch imu_complementary_filter complementary_filter.launch.py"
        "ros2 run lidar_preprocessing lidar_preprocessing"
	"ros2 launch fast_lio mapping_mid360.launch.py"
	
	)

for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done
