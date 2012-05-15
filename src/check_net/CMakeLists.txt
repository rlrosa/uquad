# Generate a to control connectivity using UDP
add_library (uquad_check_net uquad_check_net)
target_link_libraries(uquad_check_net uquad_time)
target_link_libraries(uquad_check_net uquad_io)

set(server_bin server)
add_executable (${server_bin} server)

set(client_bin client)
add_executable (${client_bin} client)

# Set required libraries
target_link_libraries(${client_bin} uquad_check_net)
target_link_libraries(${server_bin} uquad_check_net)
