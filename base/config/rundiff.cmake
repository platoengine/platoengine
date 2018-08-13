# 0. decompose mesh if parallel

string(REPLACE " " ";" NUM_PROCS_LIST ${NUM_PROCS})

message(STATUS "decomp is ${SEACAS_DECOMP}")

if (NOT SEACAS_DECOMP)
  message(FATAL_ERROR "Cannot find decomp")
endif()

foreach(NUM_PROC ${NUM_PROCS_LIST})
  SET(DECOMP_COMMAND ${SEACAS_DECOMP} -processors ${NUM_PROC} ${INPUT_MESH})
  message("Running the command:")
  message("${DECOMP_COMMAND}")
  EXECUTE_PROCESS(COMMAND ${DECOMP_COMMAND} RESULT_VARIABLE HAD_ERROR)
endforeach()

if(HAD_ERROR)
  message(FATAL_ERROR "decomp failed")
endif()



# 1. Run the program and generate the exodus output

message("Running the command:")
message("${TEST_COMMAND}")
execute_process(COMMAND bash "-c" "echo ${TEST_COMMAND}" RESULT_VARIABLE HAD_ERROR)
execute_process(COMMAND bash "-c" "${TEST_COMMAND}" OUTPUT_FILE console.log RESULT_VARIABLE HAD_ERROR)
if(HAD_ERROR)
  message(FATAL_ERROR "run failed")
endif()
execute_process(COMMAND bash "-c" "awk '/Fatal Error/,0' console.log" OUTPUT_FILE error.log RESULT_VARIABLE HAD_ERROR)
if(HAD_ERROR)
  message(FATAL_ERROR "awk failed")
endif()


# 2. diff console output with gold file

#SET(DIFF_TEST diff error.log ${DATA_DIR}/${TEST_NAME}.ref.diff)
SET(DIFF_TEST "diff <(sort error.log) <(sort ${DATA_DIR}/${TEST_NAME}.ref.diff)")

message("Running the command:")
message("${DIFF_TEST}")

#EXECUTE_PROCESS(COMMAND ${DIFF_TEST} OUTPUT_FILE diff.out RESULT_VARIABLE HAD_ERROR)
EXECUTE_PROCESS(COMMAND bash "-c" "${DIFF_TEST}" OUTPUT_FILE diff.out RESULT_VARIABLE HAD_ERROR)

if(HAD_ERROR)
  message(FATAL_ERROR "Test failed")
endif()
