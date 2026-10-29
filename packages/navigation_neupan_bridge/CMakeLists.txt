cmake_minimum_required(VERSION 3.8)
project(navigation_neupan_bridge)

find_package(ament_cmake REQUIRED)
find_package(r1_neupan_runtime REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
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
  src/soft_clearance_layer.cpp
  src/guard_aware_progress_checker.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  r1_neupan_runtime
  rcl_interfaces
  std_srvs
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

add_library(neupan_wait_condition_bt_node SHARED src/neupan_wait_condition_bt_node.cpp)
target_include_directories(neupan_wait_condition_bt_node PRIVATE include)
target_compile_features(neupan_wait_condition_bt_node PUBLIC cxx_std_17)
target_compile_definitions(neupan_wait_condition_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(neupan_wait_condition_bt_node
  behaviortree_cpp_v3 nav2_behavior_tree nav_msgs rclcpp std_msgs std_srvs r1_neupan_runtime)
find_library(NEUPAN_FOLLOW_PATH_BT_LIBRARY nav2_follow_path_action_bt_node
  PATHS "${nav2_behavior_tree_DIR}/../../../lib" REQUIRED)
target_link_libraries(neupan_wait_condition_bt_node "${NEUPAN_FOLLOW_PATH_BT_LIBRARY}")

add_executable(r1_localization_monitor src/r1_localization_monitor.cpp)
target_compile_features(r1_localization_monitor PUBLIC cxx_std_17)
target_include_directories(r1_localization_monitor PRIVATE include)
ament_target_dependencies(r1_localization_monitor
  geometry_msgs nav2_msgs nav_msgs rclcpp std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros)

add_executable(r1_arm_footprint src/r1_arm_footprint_node.cpp)
target_compile_features(r1_arm_footprint PUBLIC cxx_std_17)
ament_target_dependencies(r1_arm_footprint geometric_shapes geometry_msgs rclcpp sensor_msgs std_msgs tf2 tf2_ros)

add_executable(r1_dynamic_footprint_sync src/r1_dynamic_footprint_sync.cpp)
target_compile_features(r1_dynamic_footprint_sync PUBLIC cxx_std_17)
target_include_directories(r1_dynamic_footprint_sync PRIVATE include)
ament_target_dependencies(r1_dynamic_footprint_sync geometry_msgs rclcpp std_msgs)

add_executable(nav_lifecycle_ensure src/nav_lifecycle_ensure.cpp)
target_compile_features(nav_lifecycle_ensure PUBLIC cxx_std_17)
ament_target_dependencies(nav_lifecycle_ensure lifecycle_msgs rclcpp)

add_executable(r1_passage_clearance_diagnostics src/r1_passage_clearance_diagnostics.cpp)
target_compile_features(r1_passage_clearance_diagnostics PUBLIC cxx_std_17)
target_include_directories(r1_passage_clearance_diagnostics PRIVATE include)
ament_target_dependencies(r1_passage_clearance_diagnostics
  geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros visualization_msgs)

pluginlib_export_plugin_description_file(nav2_core navigation_neupan_bridge.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d navigation_neupan_costmap_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(TARGETS neupan_wait_condition_bt_node LIBRARY DESTINATION lib)
install(TARGETS r1_localization_monitor r1_arm_footprint r1_dynamic_footprint_sync nav_lifecycle_ensure
  r1_passage_clearance_diagnostics
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES navigation_neupan_bridge.xml navigation_neupan_costmap_plugins.xml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_library(same_uuid_route_controller SHARED test/same_uuid_route_controller.cpp)
  target_compile_features(same_uuid_route_controller PUBLIC cxx_std_17)
  target_link_libraries(same_uuid_route_controller ${PROJECT_NAME})
  ament_target_dependencies(same_uuid_route_controller nav2_core pluginlib)
  pluginlib_export_plugin_description_file(nav2_core test/same_uuid_route_plugins.xml)
  install(TARGETS same_uuid_route_controller LIBRARY DESTINATION lib)
  add_library(action_stall_controller SHARED test/action_stall_controller.cpp)
  target_compile_features(action_stall_controller PUBLIC cxx_std_17)
  target_link_libraries(action_stall_controller ${PROJECT_NAME})
  ament_target_dependencies(action_stall_controller nav2_core pluginlib std_msgs)
  pluginlib_export_plugin_description_file(nav2_core test/action_stall_plugins.xml)
  install(TARGETS action_stall_controller LIBRARY DESTINATION lib)
  add_library(controlled_current_layer SHARED test/controlled_current_layer.cpp)
  target_compile_features(controlled_current_layer PUBLIC cxx_std_17)
  ament_target_dependencies(controlled_current_layer nav2_costmap_2d pluginlib rclcpp std_msgs)
  pluginlib_export_plugin_description_file(nav2_costmap_2d test/controlled_current_plugins.xml)
  install(TARGETS controlled_current_layer LIBRARY DESTINATION lib)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_same_uuid_adapter_guard test/test_same_uuid_adapter_guard.cpp)
  target_compile_features(test_same_uuid_adapter_guard PUBLIC cxx_std_17)
  ament_target_dependencies(test_same_uuid_adapter_guard nav2_core nav2_costmap_2d pluginlib)
  ament_add_gtest(test_guard_recovery test/test_guard_recovery.cpp)
  target_include_directories(test_guard_recovery PRIVATE include)
  ament_add_gtest(test_static_map_overlap_state test/test_static_map_overlap_state.cpp)
  target_include_directories(test_static_map_overlap_state PRIVATE include)
  ament_add_gtest(test_static_map_fault_geometry test/test_static_map_fault_geometry.cpp)
  target_compile_features(test_static_map_fault_geometry PRIVATE cxx_std_17)
  target_include_directories(test_static_map_fault_geometry PRIVATE include)
  ament_add_gtest(test_polygon_boundary_distance test/test_polygon_boundary_distance.cpp)
  target_compile_features(test_polygon_boundary_distance PRIVATE cxx_std_17)
  target_include_directories(test_polygon_boundary_distance PRIVATE include)
  ament_add_gtest(test_path_replan_distance test/test_path_replan_distance.cpp)
  target_include_directories(test_path_replan_distance PRIVATE include)
  ament_target_dependencies(test_path_replan_distance geometry_msgs nav_msgs)
  ament_add_gtest(test_soft_clearance_cost test/test_soft_clearance_cost.cpp)
  target_include_directories(test_soft_clearance_cost PRIVATE include)
  ament_target_dependencies(test_soft_clearance_cost nav2_costmap_2d pluginlib)
  ament_add_gtest(test_passage_clearance_state test/test_passage_clearance_state.cpp)
  target_include_directories(test_passage_clearance_state PRIVATE include)
  ament_add_gtest(test_guard_aware_progress_checker
    test/test_guard_aware_progress_checker.cpp)
  target_link_libraries(test_guard_aware_progress_checker ${PROJECT_NAME})
  ament_target_dependencies(test_guard_aware_progress_checker nav2_core pluginlib)
  ament_add_gtest(test_neupan_execution_ros test/test_neupan_execution_ros.cpp)
  target_compile_features(test_neupan_execution_ros PUBLIC cxx_std_17)
  target_link_libraries(test_neupan_execution_ros ${PROJECT_NAME})
  ament_target_dependencies(test_neupan_execution_ros
    r1_neupan_runtime nav2_core nav2_costmap_2d nav_msgs rclcpp std_msgs std_srvs tf2_ros)
  find_library(NAV2_RECOVERY_BT_LIBRARY nav2_recovery_node_bt_node
    PATHS "${nav2_behavior_tree_DIR}/../../../lib" REQUIRED)
  ament_add_gtest(test_neupan_wait_condition_bt test/test_neupan_wait_condition_bt.cpp)
  target_compile_definitions(test_neupan_wait_condition_bt PRIVATE
    WAIT_PLUGIN_PATH="$<TARGET_FILE:neupan_wait_condition_bt_node>"
    RECOVERY_PLUGIN_PATH="${NAV2_RECOVERY_BT_LIBRARY}")
  ament_target_dependencies(test_neupan_wait_condition_bt
    behaviortree_cpp_v3 geometry_msgs nav_msgs rclcpp std_msgs std_srvs)
  ament_add_gtest(test_execution_owner_lease test/test_execution_owner_lease.cpp)
  target_include_directories(test_execution_owner_lease PRIVATE include)
  ament_target_dependencies(test_execution_owner_lease r1_neupan_runtime)
endif()

ament_export_dependencies(
  r1_neupan_runtime rcl_interfaces
  geometric_shapes geometry_msgs lifecycle_msgs nav2_core nav2_costmap_2d nav2_util nav2_msgs nav_msgs pluginlib rclcpp
  std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros visualization_msgs
)
ament_package()
