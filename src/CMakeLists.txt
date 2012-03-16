cmake_minimum_required (VERSION 2.8.3)
project (UQUAD)

# List all dirs with headers
include_directories(common)
include_directories(motor)
include_directories(kernel_msgq)
include_directories(imu)
include_directories(math)
include_directories(gps)
include_directories(submodules/gpsd)
include_directories(kalman)
include_directories(control)
include_directories(path_planner)

# List external libs
link_directories(submodules/gpsd)

# Add libm, for pow()
link_libraries(m)

# Add warning to compile flags:
set(${PROJECT_NAME}_C_FLAGS "-Wall" CACHE STRING
    "${PROJECT_NAME} C FLAGS")
# Enable the project's flags:
set(CMAKE_C_FLAGS "${${PROJECT_NAME}_C_FLAGS}")

# Generate libs
add_subdirectory(imu)
add_subdirectory(motor)
add_subdirectory(common)
add_subdirectory(math)
add_subdirectory(kernel_msgq)
add_subdirectory(gps)
add_subdirectory(kalman)
add_subdirectory(main)
add_subdirectory(control)
add_subdirectory(path_planner)

# Check in test dir for deeper includes
add_subdirectory(test)
