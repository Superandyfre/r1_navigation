cmake_minimum_required(VERSION 3.8)
project(r1_lidar_preprocess)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(scan_fusion_node src/scan_fusion_node.cpp)
target_compile_features(scan_fusion_node PUBLIC cxx_std_17)
ament_target_dependencies(scan_fusion_node
  rclcpp sensor_msgs std_msgs tf2 tf2_ros)

install(TARGETS scan_fusion_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
