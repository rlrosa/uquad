# Generate a lib for logging
# The extension is already found. Any number of sources could be listed here.
add_library (uquad_logger uquad_logger)

target_link_libraries(uquad_logger uquad_time)
target_link_libraries(uquad_logger uquad_io)

# Add an executable which will read out data from pipes
set(logger logger)
add_executable (${logger} logger)

target_link_libraries(${logger} uquad_io)
target_link_libraries(${logger} uquad_time)
