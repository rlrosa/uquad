# Writer module
set(writer_bin writer)
add_executable (${writer_bin} writer)

# Set required libraries
target_link_libraries(${writer_bin} uquad_time uquad_io)

# Reader module
set(reader_bin reader)
add_executable (${reader_bin} reader)

# Set required libraries
target_link_libraries(${reader_bin} uquad_time uquad_io)

# Forker module
set(forker_bin forker)
add_executable (${forker_bin} forker)

# Set required libraries
target_link_libraries(${forker_bin} uquad_logger)
