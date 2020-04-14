# 0. Run XMLGenerator on plato input deck

message("Running the command:")
message("${XMLGEN_COMMAND}")
execute_process(COMMAND bash "-c" "echo ${XMLGEN_COMMAND}" RESULT_VARIABLE HAD_ERROR)
execute_process(COMMAND bash "-c" "${XMLGEN_COMMAND}" RESULT_VARIABLE HAD_ERROR)

# 1. Run the program and generate the exodus output

message("Running the command:")
message("${TEST_COMMAND}")
execute_process(COMMAND bash "-c" "echo ${TEST_COMMAND}" RESULT_VARIABLE HAD_ERROR)
execute_process(COMMAND bash "-c" "${TEST_COMMAND}" RESULT_VARIABLE HAD_ERROR)

if(HAD_ERROR)
  message(FATAL_ERROR "run failed")
endif()



# 2. Find and run epu if parallel

string(REPLACE " " ";" NUM_PROCS_LIST ${NUM_PROCS})
list(GET NUM_PROCS_LIST ${IO_COMM_INDEX} NUM_PARTS)

if(${NUM_PARTS} MATCHES "1")
  set(OUTPUT_MESH ${OUTPUT_MESH}.${NUM_PARTS}.0)
  message("EPU was skipped becasue there is only one file part.")
else()
  if (NOT SEACAS_EPU)
    message(FATAL_ERROR "Cannot find epu")
  endif()

  SET(EPU_COMMAND ${SEACAS_EPU} -auto ${OUTPUT_MESH}.${NUM_PARTS}.0)

  message("Running the command:")
  message("${EPU_COMMAND}")

  EXECUTE_PROCESS(COMMAND ${EPU_COMMAND} RESULT_VARIABLE HAD_ERROR)

  if(HAD_ERROR)
    message(FATAL_ERROR "epu failed")
  endif()
endif()


# 2. Find and run exodiff

if(${SKIP_EXODIFF} MATCHES "1")
  message("----------------- Skipping exodiff ------------------")
else()
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
endif()
