set(kalman_test_bin kalman_test)
add_executable (${kalman_test_bin} kalman_test)

# Set required libraries
target_link_libraries(${kalman_test_bin} imu)
target_link_libraries(${kalman_test_bin} uquad_time)
target_link_libraries(${kalman_test_bin} uquad_math)
target_link_libraries(${kalman_test_bin} kalman)
target_link_libraries(${kalman_test_bin} uquad_io)
target_link_libraries(${kalman_test_bin} gps_comm)