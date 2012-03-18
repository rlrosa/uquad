# Generate a lib for imu communication
# The extension is already found. Any number of sources could be listed here.
add_library (kalman uquad_kalman)

add_dependencies(kalman imu)
add_dependencies(kalman uquad_math)
