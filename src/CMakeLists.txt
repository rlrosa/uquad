cmake_minimum_required (VERSION 2.8.3)
project (UQUAD)

# List all dirs with headers
include_directories(common imu math gps submodules/gpsd kalman)

# List external libs
link_directories(submodules/gpsd)

# Add libm, for pow()
link_libraries(m)

# Generate libs
add_subdirectory(imu)
add_subdirectory(common)
add_subdirectory(math)
add_subdirectory(gps)
add_subdirectory(kalman)

# Check in test dir for deeper includes
add_subdirectory(test)
