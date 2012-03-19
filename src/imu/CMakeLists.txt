# Generate a lib for imu communication
# The extension is already found. Any number of sources could be listed here.
add_library (imu imu_comm)

add_dependencies(imu uquad_time)
add_dependencies(imu uquad_math)
add_dependencies(imu uquad_io)
