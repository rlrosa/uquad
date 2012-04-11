set(timing_bin timing)
add_executable (${timing_bin}  timing)

# Set required libraries
target_link_libraries(${timing_bin} uquad_time)
target_link_libraries(${timing_bin} uquad_logger)

