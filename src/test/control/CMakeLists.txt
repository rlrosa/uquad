set(control_test_bin control_test)
add_executable (${control_test_bin} control_test)

# Set required libraries
target_link_libraries(${control_test_bin} control)
target_link_libraries(${control_test_bin} uquad_math)
target_link_libraries(${control_test_bin} path_planner)