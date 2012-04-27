# Generate a for inter-process communication using kernel queues.
# The extension is already found. Any number of sources could be listed here.
add_library (uquad_kernel_msgq uquad_kernel_msgq)

target_link_libraries(uquad_kernel_msgq uquad_time)