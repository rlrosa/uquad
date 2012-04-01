set(uquad_aux_io_test_bin uquad_aux_io_test)
add_executable (${uquad_aux_io_test_bin}  uquad_aux_io_test)

# Set required libraries
target_link_libraries(${uquad_aux_io_test_bin}  uquad_io)
target_link_libraries(${uquad_aux_io_test_bin}  imu)
target_link_libraries(${uquad_aux_io_test_bin}  uquad_logger)
target_link_libraries(${uquad_aux_io_test_bin}  uquad_time)
target_link_libraries(${uquad_aux_io_test_bin}  uquad_math)
