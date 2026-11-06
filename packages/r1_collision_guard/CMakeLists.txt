cmake_minimum_required(VERSION 3.8)
project(r1_collision_guard)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(r1_neupan_runtime REQUIRED)
execute_process(COMMAND git rev-parse --short HEAD WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE R1_FAILSTOP_BUILD_REVISION OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT R1_FAILSTOP_BUILD_REVISION)
  set(R1_FAILSTOP_BUILD_REVISION "unknown")
endif()

add_executable(${PROJECT_NAME}_node src/collision_guard_node.cpp)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_node PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}_node geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs std_srvs r1_neupan_runtime)
target_compile_definitions(${PROJECT_NAME}_node PRIVATE R1_FAILSTOP_BUILD_REVISION="${R1_FAILSTOP_BUILD_REVISION}")

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
