cmake_minimum_required(VERSION 3.10)
project(dron_tf2_transformations)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED) 
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# find_package(  REQUIRED)
ament_python_install_package(${PROJECT_NAME})


add_executable(odometry_tf_transformation src/odometry_tf_transformation.cpp)

ament_target_dependencies(odometry_tf_transformation
  rclcpp
  geometry_msgs
  nav_msgs
  Eigen3
  px4_msgs
  px4_ros_com
  tf2_ros
  tf2_geometry_msgs
)

add_executable(odom_ros_2_px4 src/odom_ros_2_px4.cpp)

ament_target_dependencies(odom_ros_2_px4
  rclcpp
  nav_msgs
  Eigen3
  px4_msgs
  px4_ros_com
)



install(TARGETS 
odometry_tf_transformation
odom_ros_2_px4
  DESTINATION lib/${PROJECT_NAME})

# Install the executable Python scripts so ros2 run can find them
install(PROGRAMS
  dron_tf2_transformations/tf2_static_broadcaster.py
  dron_tf2_transformations/tf2_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  launch
  
  DESTINATION share/${PROJECT_NAME}/
)

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
