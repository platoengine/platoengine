# 1. Run the program and generate the exodus output

message("Running the command:")
message("${TEST_COMMAND}")
execute_process(COMMAND bash "-c" "echo ${TEST_COMMAND}" RESULT_VARIABLE HAD_ERROR)
execute_process(COMMAND bash "-c" "${TEST_COMMAND}" RESULT_VARIABLE HAD_ERROR)

if(HAD_ERROR)
  message(FATAL_ERROR "run failed")
endif()

# 2. combine columns of results and gold files 

SET(PASTE_CMD "paste ${OUT_FILE} ${GOLD_FILE} > tmp")

message("Running unix paste command:")
message("${PASTE_CMD}")

execute_process(COMMAND bash "-c" "${PASTE_CMD}" RESULT_VARIABLE HAD_ERROR)
if(HAD_ERROR)
  message(FATAL_ERROR "paste failed")
endif()

# 3. Run awk comparison

SET(AWK_TEST "awk -f ${AWK_FILE} tmp")

message("Running unix-awk command:")
message("${AWK_TEST}")

execute_process(COMMAND bash "-c" "${AWK_TEST}" RESULT_VARIABLE HAD_ERROR)

if(HAD_ERROR)
  message(FATAL_ERROR "Test failed")
endif()
