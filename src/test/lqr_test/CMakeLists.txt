set(exp_test_bin exp_test)
add_executable (${exp_test_bin} exp_test)

# Set required libraries
target_link_libraries(${exp_test_bin} uquad_math)
