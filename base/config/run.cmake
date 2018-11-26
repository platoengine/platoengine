foreach(COMMAND ${COMMANDS})
  message("Running the command:")
  message("${COMMAND}")
  execute_process(COMMAND bash "-c" "${COMMAND}" RESULT_VARIABLE HAD_ERROR)

  if(HAD_ERROR)
    message(FATAL_ERROR "command failed")
  endif()
endforeach()
