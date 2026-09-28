cmake_minimum_required(VERSION 3.8)
project(r1_collision_guard)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}_logic SHARED src/guard_logic.cpp)
target_compile_features(${PROJECT_NAME}_logic PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(${PROJECT_NAME}_node src/collision_guard_node.cpp)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_logic)
ament_target_dependencies(${PROJECT_NAME}_node geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs)

install(TARGETS ${PROJECT_NAME}_logic ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_guard_logic test/test_guard_logic.cpp)
  target_link_libraries(test_guard_logic ${PROJECT_NAME}_logic)
  target_include_directories(test_guard_logic PRIVATE include)
endif()

ament_package()
