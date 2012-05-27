# Generate a lib for interaction with motor driver.
# The extension is already found. Any number of sources could be listed here.
add_library (control control)

target_link_libraries(control uquad_math)
target_link_libraries(control uquad_time)
