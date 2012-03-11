set(motor_test_bin motor_test)
add_executable (${motor_test_bin} motor_test)

# Set required libraries
target_link_libraries(${motor_test_bin} motor uquad_math uquad_time uquad_kernel_msgq)
