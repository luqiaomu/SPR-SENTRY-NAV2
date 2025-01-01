cmake_minimum_required(VERSION 3.8)
project(pcl_icp_localization)


set(CMAKE_BUILD_TYPE Release)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
# 找到OpenMP包
find_package(OpenMP REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(pcl_icp src/pcl_icp_localization.cpp)

# 链接依赖项
ament_target_dependencies(pcl_icp
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
  tf2_ros
  geometry_msgs
  nav_msgs
)

# 链接PCL库
target_link_libraries(pcl_icp
  ${PCL_LIBRARIES}
)

# 链接OpenMP库
if(OpenMP_CXX_FOUND)
  target_link_libraries(pcl_icp OpenMP::OpenMP_CXX)
endif()

# 安装目标
install(TARGETS pcl_icp
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
