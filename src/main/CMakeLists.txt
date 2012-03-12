set(main_bin main)
add_executable (${main_bin} main)

# Set required libraries
target_link_libraries(${main_bin} uquad_math imu uquad_aux_io uquad_aux_time uquad_aux_log uquad_gps_comm)
