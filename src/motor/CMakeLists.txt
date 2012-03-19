# Generate a lib for motor control.
# Will drive the program running on the beagleboard.
# The extension is already found. Any number of sources could be listed here.
add_library (motor mot_control)

add_dependencies(motor uquad_time)
add_dependencies(motor uquad_math)
add_dependencies(motor uquad_kernel_msgq)
