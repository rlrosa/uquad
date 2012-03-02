set(math_test_bin math_test)
add_executable (${math_test_bin} math_test)

# Set required libraries
target_link_libraries(${math_test_bin} uquad_math)
