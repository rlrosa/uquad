# send pgm uses uquad lib
set(q_send_bin message_send)
add_executable (${q_send_bin} message_send)

# Set required libraries
target_link_libraries(${q_send_bin} uquad_time uquad_kernel_msgq)

set(q_rec_bin message_rec)
add_executable (${q_rec_bin} message_rec)

# Set required libraries
target_link_libraries(${q_rec_bin} uquad_time)
