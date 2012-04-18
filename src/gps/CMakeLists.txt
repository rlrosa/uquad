# Generate a lib for gpsd interface
# The extension is already found. Any number of sources could be listed here.
add_library (gps_comm uquad_gps_comm)

target_link_libraries(gps_comm gps)
target_link_libraries(gps_comm uquad_time)
target_link_libraries(gps_comm uquad_io)
target_link_libraries(gps_comm uquad_math)
