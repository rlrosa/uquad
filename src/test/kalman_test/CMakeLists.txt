set(kalman_test_bin kalman_test)
add_executable (${kalman_test_bin} kalman_test)

# Set required libraries
target_link_libraries(${kalman_test_bin} imu uquad_time uquad_math kalman)
