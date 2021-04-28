# 1. Run the program and generate the exodus output

message("Running the command:")
message("${TEST_COMMAND}")
execute_process(COMMAND bash "-c" "echo ${TEST_COMMAND}" RESULT_VARIABLE HAD_ERROR)
execute_process(COMMAND bash "-c" "${TEST_COMMAND}" RESULT_VARIABLE HAD_ERROR)

if(HAD_ERROR)
  message(FATAL_ERROR "run failed")
endif()
