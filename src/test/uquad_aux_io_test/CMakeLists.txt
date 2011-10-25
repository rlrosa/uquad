set(uquad_aux_io_test_bin uquad_aux_io_test)
add_executable (${uquad_aux_io_test_bin}  uquad_aux_io_test)

# Set required libraries
target_link_libraries(${uquad_aux_io_test_bin}  uquad_io imu uquad_log uquad_time)
