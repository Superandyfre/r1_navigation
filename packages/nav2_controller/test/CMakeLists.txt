# Test dynamic parameters
ament_add_gtest(test_dynamic_parameters
  test_dynamic_parameters.cpp
)
ament_target_dependencies(test_dynamic_parameters
  ${dependencies}
)
target_link_libraries(test_dynamic_parameters
  ${library_name}
)

ament_add_gtest(test_execution_lifecycle test_execution_lifecycle.cpp)
ament_target_dependencies(test_execution_lifecycle ${dependencies})
target_link_libraries(test_execution_lifecycle ${library_name} simple_goal_checker simple_progress_checker)
