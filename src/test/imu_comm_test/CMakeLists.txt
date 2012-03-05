set(imu_comm_test_bin imu_comm_test)
add_executable (${imu_comm_test_bin} imu_comm_test)

# Set required libraries
target_link_libraries(${imu_comm_test_bin} imu uquad_time uquad_math)
