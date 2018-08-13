# 1. Run the program and generate the exodus output

message("Running the command:")
message("${TEST_COMMAND}")
execute_process(COMMAND bash "-c" "${TEST_COMMAND}" RESULT_VARIABLE HAD_ERROR)

if(HAD_ERROR)
  message(FATAL_ERROR "run failed")
endif()

# 2. Find and run exodiff

if (NOT SEACAS_EXODIFF)
  message(FATAL_ERROR "Cannot find exodiff")
endif()

SET(EXODIFF_TEST ${SEACAS_EXODIFF} -i -m -f ${DATA_DIR}/${TEST_NAME}.exodiff_commands ${OUTPUT_MESH} ${DATA_DIR}/${TEST_NAME}.ref.exo)

message("Running the command:")
message("${EXODIFF_TEST}")

EXECUTE_PROCESS(COMMAND ${EXODIFF_TEST} OUTPUT_FILE exodiff.out RESULT_VARIABLE HAD_ERROR)

if(HAD_ERROR)
  message(FATAL_ERROR "Test failed")
endif()
