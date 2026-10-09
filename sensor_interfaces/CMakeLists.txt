cmake_minimum_required(VERSION 3.8)
project(sensor_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# Ejecutables
add_executable(imu_converter_px4_ros_node src/imu_converter_px4_ros_node.cpp)
ament_target_dependencies(imu_converter_px4_ros_node
  rclcpp
  px4_msgs
  px4_ros_com
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
)

add_executable(odom_republisher src/odom_republisher.cpp)
ament_target_dependencies(odom_republisher
  rclcpp
  geometry_msgs
  nav_msgs
  px4_msgs
  px4_ros_com
  tf2_ros
  tf2_geometry_msgs
)

add_executable(teensy_serial_receiver src/teensy_serial_receiver.cpp)
ament_target_dependencies(teensy_serial_receiver
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
)

target_link_libraries(teensy_serial_receiver
  ${PCL_LIBRARIES}
  ${LIBSERIAL_LIBRARIES}
)

target_include_directories(teensy_serial_receiver
  PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${LIBSERIAL_INCLUDE_DIRS}
)

target_compile_definitions(teensy_serial_receiver
  PRIVATE
    ${PCL_DEFINITIONS}
)

# Instalación
install(TARGETS imu_converter_px4_ros_node odom_republisher teensy_serial_receiver
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()