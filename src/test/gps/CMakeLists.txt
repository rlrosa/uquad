set(gps_test_bin gps_test)
add_executable (${gps_test_bin} gps_test)

# Set required libraries
target_link_libraries(${gps_test_bin} gps_comm)
target_link_libraries(${gps_test_bin} uquad_io)
target_link_libraries(${gps_test_bin} uquad_math)
