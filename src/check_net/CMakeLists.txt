set(client_bin client)
add_executable (${client_bin} client)

set(server_bin server)
add_executable (${server_bin} server)

# Set required libraries
target_link_libraries(${client_bin} uquad_time)
target_link_libraries(${client_bin} uquad_io)
