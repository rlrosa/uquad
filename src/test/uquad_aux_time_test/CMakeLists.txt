set(uquad_aux_time_test_bin uquad_aux_time_test)
add_executable (${uquad_aux_time_test_bin}  uquad_aux_time_test)

# Set required libraries
target_link_libraries(${uquad_aux_time_test_bin}  uquad_time)
