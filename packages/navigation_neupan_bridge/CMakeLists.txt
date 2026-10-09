cmake_minimum_required(VERSION 3.8)
project(navigation_neupan_bridge)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/neupan_controller.cpp
  src/current_footprint_clear_layer.cpp
  src/guard_aware_progress_checker.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav2_msgs
  nav_msgs
  pluginlib
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_executable(r1_localization_monitor src/r1_localization_monitor.cpp)
target_compile_features(r1_localization_monitor PUBLIC cxx_std_17)
ament_target_dependencies(r1_localization_monitor
  geometry_msgs nav2_msgs nav_msgs rclcpp std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros)

add_executable(r1_arm_footprint src/r1_arm_footprint_node.cpp)
target_compile_features(r1_arm_footprint PUBLIC cxx_std_17)
ament_target_dependencies(r1_arm_footprint geometric_shapes geometry_msgs rclcpp sensor_msgs std_msgs tf2 tf2_ros)

add_executable(r1_dynamic_footprint_sync src/r1_dynamic_footprint_sync.cpp)
target_compile_features(r1_dynamic_footprint_sync PUBLIC cxx_std_17)
ament_target_dependencies(r1_dynamic_footprint_sync geometry_msgs rclcpp std_msgs)

add_executable(nav_lifecycle_ensure src/nav_lifecycle_ensure.cpp)
target_compile_features(nav_lifecycle_ensure PUBLIC cxx_std_17)
ament_target_dependencies(nav_lifecycle_ensure lifecycle_msgs rclcpp)

add_executable(r1_passage_clearance_diagnostics src/r1_passage_clearance_diagnostics.cpp)
target_compile_features(r1_passage_clearance_diagnostics PUBLIC cxx_std_17)
ament_target_dependencies(r1_passage_clearance_diagnostics
  geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros visualization_msgs)

pluginlib_export_plugin_description_file(nav2_core navigation_neupan_bridge.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d navigation_neupan_costmap_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS r1_localization_monitor r1_arm_footprint r1_dynamic_footprint_sync nav_lifecycle_ensure
  r1_passage_clearance_diagnostics
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES navigation_neupan_bridge.xml navigation_neupan_costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_guard_recovery test/test_guard_recovery.cpp)
  target_include_directories(test_guard_recovery PRIVATE include)
  ament_add_gtest(test_path_replan_distance test/test_path_replan_distance.cpp)
  target_include_directories(test_path_replan_distance PRIVATE include)
  ament_target_dependencies(test_path_replan_distance geometry_msgs nav_msgs)
  ament_add_gtest(test_guard_aware_progress_checker
    test/test_guard_aware_progress_checker.cpp)
  target_link_libraries(test_guard_aware_progress_checker ${PROJECT_NAME})
  ament_target_dependencies(test_guard_aware_progress_checker nav2_core pluginlib)
endif()

ament_export_dependencies(
  geometric_shapes geometry_msgs lifecycle_msgs nav2_core nav2_costmap_2d nav2_util nav2_msgs nav_msgs pluginlib rclcpp
  std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros visualization_msgs
)
ament_package()
