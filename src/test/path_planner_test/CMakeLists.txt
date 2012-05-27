set(path_planner_test_bin path_planner_test)
add_executable (${path_planner_test_bin} path_planner_test)

# Set required libraries
target_link_libraries(${path_planner_test_bin} path_planner)
target_link_libraries(${path_planner_test_bin} control)
target_link_libraries(${path_planner_test_bin} uquad_math)
