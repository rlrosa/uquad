set(lqr_test_bin lqr_test)
add_executable (${lqr_test_bin} lqr_test)

# Set required libraries
target_link_libraries(${lqr_test_bin} uquad_math)
target_link_libraries(${lqr_test_bin} path_planner)
target_link_libraries(${lqr_test_bin} control)
