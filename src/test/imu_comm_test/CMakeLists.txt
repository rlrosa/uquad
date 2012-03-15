set(imu_comm_test_bin imu_comm_test)
add_executable (${imu_comm_test_bin} imu_comm_test)

# Set required libraries
target_link_libraries(${imu_comm_test_bin} imu)
target_link_libraries(${imu_comm_test_bin} uquad_time)
target_link_libraries(${imu_comm_test_bin} uquad_math)
target_link_libraries(${imu_comm_test_bin} uquad_log)
