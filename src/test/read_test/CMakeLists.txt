# send pgm uses uquad lib
set(read_only_bin read_only)
add_executable (${read_only_bin} read_only)

# Set required libraries
target_link_libraries(${read_only_bin} uquad_time uquad_io)
