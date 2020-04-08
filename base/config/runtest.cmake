# 0. decompose mesh if parallel

string(REPLACE " " ";" NUM_PROCS_LIST ${NUM_PROCS})

message(STATUS "decomp is ${SEACAS_DECOMP}")

if (NOT SEACAS_DECOMP)
  message(FATAL_ERROR "Cannot find decomp")
endif()

if (RESTART_MESH)
  foreach(NUM_PROC ${NUM_PROCS_LIST})
    SET(DECOMP_COMMAND ${SEACAS_DECOMP} -processors ${NUM_PROC} ${RESTART_MESH})
    message("Running the command:")
    message("${DECOMP_COMMAND}")
    EXECUTE_PROCESS(COMMAND ${DECOMP_COMMAND} RESULT_VARIABLE HAD_ERROR)
  endforeach()
endif()

if(NOT ${INPUT_MESH} STREQUAL SKIP_DECOMP)
  foreach(NUM_PROC ${NUM_PROCS_LIST})
    SET(DECOMP_COMMAND ${SEACAS_DECOMP} -processors ${NUM_PROC} ${INPUT_MESH})
    message("Running the command:")
    message("${DECOMP_COMMAND}")
    EXECUTE_PROCESS(COMMAND ${DECOMP_COMMAND} RESULT_VARIABLE HAD_ERROR)
  endforeach()

  if(HAD_ERROR)
    message(FATAL_ERROR "decomp failed")
  endif()
endif()



# 1. Run the program and generate the exodus output

message("Running the command:")
message("${TEST_COMMAND}")
execute_process(COMMAND bash "-c" "echo ${TEST_COMMAND}" RESULT_VARIABLE HAD_ERROR)
execute_process(COMMAND bash "-c" "${TEST_COMMAND}" RESULT_VARIABLE HAD_ERROR)

if(HAD_ERROR)
  message(FATAL_ERROR "run failed")
endif()



# 2. Find and run epu if parallel

if(NOT ${OUTPUT_MESH} STREQUAL SKIP_EXODIFF)
  list(GET NUM_PROCS_LIST ${IO_COMM_INDEX} NUM_PARTS)

  if(${NUM_PARTS} MATCHES "1")
    message("EPU was skipped because there is only one file part.")
    if(EXISTS ${OUTPUT_MESH}.${NUM_PARTS}.0)
      file(RENAME ${OUTPUT_MESH}.${NUM_PARTS}.0 ${OUTPUT_MESH})
    endif()
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
