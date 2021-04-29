###############################################################################
## Plato_find_exe( 
##    VAR_NAME     == Return variable containing filepath to executable.
##    EXE_NAME     == Filename of executable.
##   [SEARCH_PATH] == Directories below this path are searched.
## )
###############################################################################
function( Plato_find_exe VAR_NAME EXE_NAME SEARCH_PATH )

  message(STATUS " ")
  message(STATUS "Finding ${EXE_NAME} executable")

  message("-- searching in " ${SEARCH_PATH})
  find_program( ${VAR_NAME}_SEARCH_RESULT ${EXE_NAME}
                HINTS ${SEARCH_PATH}
                NO_DEFAULT_PATH )

  if( ${VAR_NAME}_SEARCH_RESULT MATCHES "NOTFOUND" )
    message(FATAL_ERROR "!! ${EXE_NAME} executable not found !!")
  endif( ${VAR_NAME}_SEARCH_RESULT MATCHES "NOTFOUND" )

  set( ${VAR_NAME} ${${VAR_NAME}_SEARCH_RESULT} PARENT_SCOPE )
  message(STATUS "${EXE_NAME} executable found")
  message(STATUS "Using:  ${${VAR_NAME}_SEARCH_RESULT}")
  message(STATUS " ")

endfunction(Plato_find_exe)
###############################################################################



###############################################################################
## Plato_find_lib( 
##    VAR_NAME      == Return variable containing filepath to library.
##    OPTION_NAME   == If ON, function attempts to find requested library.
##    LIB_BASE_NAME == Basename of library.
##    SEARCH_PATH   == Directories below this path are searched.
##   [PROPER_NAME]  == Alternate name.  Used for message output only.
## )
###############################################################################

function( Plato_find_lib VAR_NAME OPTION_NAME LIB_BASE_NAME SEARCH_PATH )

  if( ${OPTION_NAME} )

message(STATUS "The search path is:  ${SEARCH_PATH} ") 

  if( ARGN GREATER 0 )  ## if optional argument included
    set(OUT_NAME ${ARGV0})
  else( ARGN GREATER 0 )  ## otherwise
    set(OUT_NAME ${LIB_BASE_NAME})
  endif( ARGN GREATER 0 )
  
  message(STATUS " ")
  message(STATUS "Finding ${OUT_NAME} executable")
  
  find_library( ${VAR_NAME}_SEARCH_RESULT ${LIB_BASE_NAME}
                HINTS ${SEARCH_PATH}
                DOC "${OUT_NAME} library"
                NO_DEFAULT_PATH )
  
  if( ${VAR_NAME}_SEARCH_RESULT MATCHES "NOTFOUND" )
    message(FATAL_ERROR "!! ${OUT_NAME} library not found !!")
  endif( ${VAR_NAME}_SEARCH_RESULT MATCHES "NOTFOUND" )
  
  set( ${VAR_NAME} ${${VAR_NAME}_SEARCH_RESULT} PARENT_SCOPE )
  message(STATUS "${OUT_NAME} library found")
  message(STATUS "Using:  ${${VAR_NAME}_SEARCH_RESULT}")
  message(STATUS " ")
  
  endif( ${OPTION_NAME} )
    
endfunction(Plato_find_lib)

###############################################################################
## Plato_find_path( 
##    VAR_NAME      == Return variable containing filepath to file
##    OPTION_NAME   == If ON, function attempts to find requested file
##    FILE_NAME     == name of file to be found
##    SEARCH_PATH   == Directories below this path are searched.
## )
###############################################################################

function( Plato_find_path VAR_NAME OPTION_NAME FILE_NAME SEARCH_PATH )

  if( ${OPTION_NAME} )

message(STATUS "The search path is:  ${SEARCH_PATH} ") 

  message(STATUS " ")
  message(STATUS "Finding ${OUT_NAME}")
  
  find_path( ${VAR_NAME}_SEARCH_RESULT ${FILE_NAME}
                HINTS ${SEARCH_PATH}
                DOC "${FILE_NAME} file"
                NO_DEFAULT_PATH )
  
  if( ${VAR_NAME}_SEARCH_RESULT MATCHES "NOTFOUND" )
    message(FATAL_ERROR "!! ${FILE_NAME} not found !!")
  endif( ${VAR_NAME}_SEARCH_RESULT MATCHES "NOTFOUND" )
  
  set( ${VAR_NAME} ${${VAR_NAME}_SEARCH_RESULT} PARENT_SCOPE )
  message(STATUS "${FILE_NAME} found")
  message(STATUS "Using:  ${${VAR_NAME}_SEARCH_RESULT}")
  message(STATUS " ")
  
  endif( ${OPTION_NAME} )
    
endfunction(Plato_find_path)

###############################################################################
function( Plato_no_src_build )

if("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
  message(STATUS " ")
  message(STATUS "In-source builds are not allowed.")
  message(STATUS "Please remove CMakeCache.txt and the CMakeFiles/ directory and then build out-of-source.")
  message(STATUS "(That is, create a build directory below the source directory and build from there.)" )
  message(STATUS " ")
  message(FATAL_ERROR " ")
endif()

endfunction( Plato_no_src_build )


###############################################################################
## Plato_add_text_to_file( 
##    FILE_LIST    == Return variable containing filepath to executable.
## )
###############################################################################

function( Plato_add_text_to_file FILE_TO_MODIFY STRING_TO_ADD )
  
  file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/${FILE_TO_MODIFY} ${STRING_TO_ADD})
    
endfunction(Plato_add_text_to_file)


###############################################################################
## Plato_add_test_files( 
##    FILE_LIST    == Return variable containing filepath to executable.
## )
###############################################################################

function( Plato_add_test_files FILE_LIST )
  
  foreach( testFile ${FILE_LIST} )
  
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${testFile} 
                   ${CMAKE_CURRENT_BINARY_DIR}/${testFile} COPYONLY)
    
  endforeach(testFile)
    
endfunction(Plato_add_test_files)

###############################################################################
## Plato_create_test
#     RUN_COMMAND       == beginning of mpirun statement for MPMD configuration
#     PLATOMAIN_BINARY  == PlatoMain executable
#     PLATOMAIN_NPROCS  == number of processes for PlatoMain
#     INTERFACE_FILE    == Plato interface file
#     APPFILE           == PlatoMain applicaiton file
#     INPUTFILE         == PlatoMain input file
###############################################################################

function( Plato_create_test RUN_COMMAND PLATOMAIN_BINARY PLATOMAIN_NPROCS INTERFACE_FILE APPFILE INPUTFILE )

  set( ${RUN_COMMAND} 
       "mpirun -np ${PLATOMAIN_NPROCS} --oversubscribe -x PLATO_PERFORMER_ID=0 -x PLATO_INTERFACE_FILE=${INTERFACE_FILE} -x PLATO_APP_FILE=${APPFILE} ${PLATOMAIN_BINARY} ${INPUTFILE}" PARENT_SCOPE )

endfunction( Plato_create_test )

###############################################################################
## Plato_create_simple_test
#     RUN_COMMAND       == beginning of mpirun statement for MPMD configuration
#     PLATOMAIN_BINARY  == PlatoMain executable
#     PLATOMAIN_NPROCS  == number of processes for PlatoMain
#     INTERFACE_FILE    == Plato interface file
###############################################################################

function( Plato_create_simple_test RUN_COMMAND PLATOMAIN_BINARY PLATOMAIN_NPROCS INTERFACE_FILE )

  set( ${RUN_COMMAND} 
       "mpirun -np ${PLATOMAIN_NPROCS} --oversubscribe -x PLATO_PERFORMER_ID=0 -x PLATO_INTERFACE_FILE=${INTERFACE_FILE} ${PLATOMAIN_BINARY}" PARENT_SCOPE )

endfunction( Plato_create_simple_test )

###############################################################################
## Plato_add_performer
#     RUN_COMMAND        == mpirun statement for MPMD configuration
#     PERFORMER_BINARY   == Performer executable
#     PERFORMER_NPROCS   == number of processes for this Performer
#     LOCAL_PERFORMER_ID == Performer ID for this performer
#     INTERFACE_FILE     == Plato interface file
#     APPFILE            == Performer applicaiton file
#     INPUTFILE          == Performer input file
###############################################################################

function( Plato_add_performer RUN_COMMAND PERFORMER_BINARY PERFORMER_NPROCS LOCAL_PERFORMER_ID INTERFACE_FILE APPFILE INPUTFILE )

  set( ${RUN_COMMAND} "${${RUN_COMMAND}} : -np ${PERFORMER_NPROCS} --oversubscribe -x PLATO_PERFORMER_ID=${LOCAL_PERFORMER_ID} -x PLATO_INTERFACE_FILE=${INTERFACE_FILE} -x PLATO_APP_FILE=${APPFILE} ${PERFORMER_BINARY} ${INPUTFILE}" PARENT_SCOPE )

endfunction( Plato_add_performer )

###############################################################################
## Plato_add_simple_performer
#     RUN_COMMAND        == mpirun statement for MPMD configuration
#     PERFORMER_BINARY   == Performer executable
#     PERFORMER_NPROCS   == number of processes for this Performer
#     LOCAL_PERFORMER_ID == Performer ID for this performer
#     INTERFACE_FILE     == Plato interface file
###############################################################################

function( Plato_add_simple_performer RUN_COMMAND PERFORMER_BINARY PERFORMER_NPROCS LOCAL_PERFORMER_ID INTERFACE_FILE )

  set( ${RUN_COMMAND} "${${RUN_COMMAND}} : -np ${PERFORMER_NPROCS} --oversubscribe -x PLATO_PERFORMER_ID=${LOCAL_PERFORMER_ID} -x PLATO_INTERFACE_FILE=${INTERFACE_FILE} ${PERFORMER_BINARY}" PARENT_SCOPE )

endfunction( Plato_add_simple_performer )

###############################################################################
## Plato_add_test( 
##    TESTED_CODE    == code to be tested
##    TEST_LENGTH    == short or long
##    TEST_NAME      == test name
##    N_PROCS        == number of processors to use for test
##    INPUT_FILE     == input file name
##    INPUT_MESH     == input mesh file name
##    OUTPUT_MESH    == output mesh file name
##   [RESTART_MESH]  == restart mesh file name (optional)
## )
###############################################################################

function( Plato_add_test RUN_COMMANDS TEST_NAME NUM_PROCS IO_COMM_INDEX INPUT_MESH OUTPUT_MESH )

    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/mpirun.source "${RUN_COMMANDS}")

    set(OptionalArgs ${ARGN})
    list(LENGTH OptionalArgs NumOptionalArgs)
    if(NumOptionalArgs GREATER 0)
      list(GET OptionalArgs 0 RESTART_MESH)
      add_test(NAME ${TEST_NAME}
             COMMAND ${CMAKE_COMMAND}
             "-DTEST_COMMANDS=${RUN_COMMANDS}"
             -DTEST_NAME=${TEST_NAME}
             -DNUM_PROCS=${NUM_PROCS}
             -DSEACAS_EPU=${SEACAS_EPU}
             -DSEACAS_EXODIFF=${SEACAS_EXODIFF}
             -DSEACAS_DECOMP=${SEACAS_DECOMP}
             -DDATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}
             -DIO_COMM_INDEX=${IO_COMM_INDEX}
             -DINPUT_MESH=${INPUT_MESH}
             -DOUTPUT_MESH=${OUTPUT_MESH}
             -DRESTART_MESH=${RESTART_MESH}
             -P ${CMAKE_SOURCE_DIR}/base/config/runtest.cmake)
    else()
      add_test(NAME ${TEST_NAME}
             COMMAND ${CMAKE_COMMAND}
             "-DTEST_COMMANDS=${RUN_COMMANDS}"
             -DTEST_NAME=${TEST_NAME}
             -DNUM_PROCS=${NUM_PROCS}
             -DSEACAS_EPU=${SEACAS_EPU}
             -DSEACAS_EXODIFF=${SEACAS_EXODIFF}
             -DSEACAS_DECOMP=${SEACAS_DECOMP}
             -DDATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}
             -DIO_COMM_INDEX=${IO_COMM_INDEX}
             -DINPUT_MESH=${INPUT_MESH}
             -DOUTPUT_MESH=${OUTPUT_MESH}
             -P ${CMAKE_SOURCE_DIR}/base/config/runtest.cmake)
    endif()

    set_tests_properties(${TEST_NAME} PROPERTIES REQUIRED_FILES "${SEACAS_EPU};${SEACAS_EXODIFF}")

endfunction( Plato_add_test )

###############################################################################
## Plato_add_xmlgen_test( 
## )
###############################################################################

function( Plato_add_xmlgen_test TEST_NAME XMLGEN_COMMAND NUM_PROCS IO_COMM_INDEX OUTPUT_MESH SKIP_EXODIFF )

#    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/mpirun.source ${RUN_COMMAND})
  set( RUN_COMMAND "source ${CMAKE_CURRENT_BINARY_DIR}/mpirun.source" )

    add_test(NAME ${TEST_NAME}
           COMMAND ${CMAKE_COMMAND} 
           -DTEST_COMMAND=${RUN_COMMAND}
           -DTEST_NAME=${TEST_NAME} 
           -DNUM_PROCS=${NUM_PROCS}
           -DXMLGEN_COMMAND=${XMLGEN_COMMAND}
           -DSKIP_EXODIFF=${SKIP_EXODIFF}
           -DSEACAS_EPU=${SEACAS_EPU} 
           -DSEACAS_EXODIFF=${SEACAS_EXODIFF} 
           -DSEACAS_DECOMP=${SEACAS_DECOMP}
           -DDATA_DIR=${CMAKE_CURRENT_SOURCE_DIR} 
           -DIO_COMM_INDEX=${IO_COMM_INDEX}
           -DOUTPUT_MESH=${OUTPUT_MESH}
           -P ${CMAKE_SOURCE_DIR}/base/config/runxmlgentest.cmake)

    set_tests_properties(${TEST_NAME} PROPERTIES REQUIRED_FILES "${SEACAS_EPU};${SEACAS_EXODIFF}")

endfunction( Plato_add_xmlgen_test )

###############################################################################
## Plato_add_single_test( 
##    TEST_NAME    == test name
##    TEST_COMMAND == command string to be passed to shell
## )
###############################################################################

function( Plato_add_single_test RUN_COMMAND TEST_NAME )

    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/mpirun.source ${RUN_COMMAND})

    add_test( NAME ${TEST_NAME}
              COMMAND ${CMAKE_COMMAND}
              -DTEST_COMMAND=${RUN_COMMAND}
              -P ${CMAKE_SOURCE_DIR}/base/config/runsingletest.cmake )

endfunction( Plato_add_single_test )

###############################################################################
## Plato_add_simple_test( 
##    TEST_NAME      == test name
##    NUM_PROCS      == number of processors to use for test
##    IO_COMM_INDEX  == io communicator index
## )
###############################################################################

function( Plato_add_simple_test RUN_COMMAND TEST_NAME NUM_PROCS IO_COMM_INDEX )

    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/mpirun.source ${RUN_COMMAND})

    add_test( NAME ${TEST_NAME}
              COMMAND ${CMAKE_COMMAND} 
              -DTEST_COMMAND=${RUN_COMMAND}
              -DDATA_DIR=${CMAKE_CURRENT_SOURCE_DIR} 
              -DOUT_FILE=${OUT_FILE} 
              -DGOLD_FILE=${GOLD_FILE} 
              -P ${CMAKE_SOURCE_DIR}/base/config/runsimpletest.cmake )

endfunction( Plato_add_simple_test )

###############################################################################
## Plato_add_awk_test( 
##    TEST_NAME      == test name
##    NUM_PROCS      == number of processors to use for test
##    IO_COMM_INDEX  == io communicator index
## )
###############################################################################

function( Plato_add_awk_test RUN_COMMAND TEST_NAME NUM_PROCS IO_COMM_INDEX )

    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/mpirun.source ${RUN_COMMAND})

    add_test( NAME ${TEST_NAME}
              COMMAND ${CMAKE_COMMAND} 
              -DTEST_COMMAND=${RUN_COMMAND}
              -DDATA_DIR=${CMAKE_CURRENT_SOURCE_DIR} 
              -DOUT_FILE=${OUT_FILE} 
              -DGOLD_FILE=${GOLD_FILE} 
              -DAWK_FILE=${AWK_FILE} 
              -P ${CMAKE_SOURCE_DIR}/base/config/runawktest.cmake )

endfunction( Plato_add_awk_test )

###############################################################################
## Plato_add_pso_test( 
##    TESTED_CODE    == code to be tested
##    TEST_LENGTH    == short or long
##    TEST_NAME      == test name
##    NUM_PROCS      == number of processors to use for test
##    IO_COMM_INDEX  == io communicator index
## )
###############################################################################

function( Plato_add_pso_test RUN_COMMAND TEST_NAME NUM_PROCS IO_COMM_INDEX )

    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/mpirun.source ${RUN_COMMAND})

    add_test( NAME ${TEST_NAME}
              COMMAND ${CMAKE_COMMAND} 
              -DTEST_COMMAND=${RUN_COMMAND}
              -DDATA_DIR=${CMAKE_CURRENT_SOURCE_DIR} 
              -DOUT_FILE=${OUT_FILE} 
              -DGOLD_FILE=${GOLD_FILE} 
              -P ${CMAKE_SOURCE_DIR}/base/config/runpsotest.cmake )

endfunction( Plato_add_pso_test )


###############################################################################
## Plato_add_diff( 
## )
###############################################################################

function( Plato_add_diff RUN_COMMAND TEST_NAME NUM_PROCS INPUT_MESH )

    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/mpirun.source ${RUN_COMMAND})

    add_test(NAME ${TEST_NAME}
           COMMAND ${CMAKE_COMMAND} 
           -DTEST_COMMAND=${RUN_COMMAND}
           -DTEST_NAME=${TEST_NAME} 
           -DNUM_PROCS=${NUM_PROCS}
           -DSEACAS_EPU=${SEACAS_EPU} 
           -DSEACAS_DECOMP=${SEACAS_DECOMP}
           -DDATA_DIR=${CMAKE_CURRENT_SOURCE_DIR} 
           -DINPUT_MESH=${INPUT_MESH}
           -P ${CMAKE_SOURCE_DIR}/base/config/rundiff.cmake)

    set_tests_properties(${TEST_NAME} PROPERTIES REQUIRED_FILES "${SEACAS_EPU}")

endfunction( Plato_add_diff )

###############################################################################
## Plato_add_serial_test( 
## )
###############################################################################

function( Plato_add_serial_test RUN_COMMAND TEST_NAME OUTPUT_MESH )

    add_test(NAME ${TEST_NAME}
           COMMAND ${CMAKE_COMMAND} 
           -DTEST_COMMAND=${RUN_COMMAND}
           -DTEST_NAME=${TEST_NAME} 
           -DDATA_DIR=${CMAKE_CURRENT_SOURCE_DIR} 
           -DOUTPUT_MESH=${OUTPUT_MESH}
           -DSEACAS_EXODIFF=${SEACAS_EXODIFF} 
           -P ${CMAKE_SOURCE_DIR}/base/config/runtest_serial.cmake)

endfunction( Plato_add_serial_test )

###############################################################################
## Plato_add_execution( 
## )
###############################################################################

function( Plato_add_execution RUN_COMMAND TEST_NAME NUM_PROCS IO_COMM_INDEX CUBIT_JOURNAL GENERATED_MESH OUTPUT_MESH )

    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/mpirun.source ${RUN_COMMAND})

    add_test(NAME ${TEST_NAME}
           COMMAND ${CMAKE_COMMAND} 
           -DTEST_COMMAND=${RUN_COMMAND}
           -DTEST_NAME=${TEST_NAME} 
           -DNUM_PROCS=${NUM_PROCS}
           -DSEACAS_EPU=${SEACAS_EPU} 
           -DSEACAS_DECOMP=${SEACAS_DECOMP}
           -DDATA_DIR=${CMAKE_CURRENT_SOURCE_DIR} 
           -DIO_COMM_INDEX=${IO_COMM_INDEX}
           -DCUBIT_JOURNAL=${CUBIT_JOURNAL}
           -DGENERATED_MESH=${GENERATED_MESH}
           -DOUTPUT_MESH=${OUTPUT_MESH}
           -P ${CMAKE_SOURCE_DIR}/base/config/runexecution.cmake)

    set_tests_properties(${TEST_NAME} PROPERTIES REQUIRED_FILES "${SEACAS_EPU}")

endfunction( Plato_add_execution )

###############################################################################
## Plato_new_test( 
##    TEST_NAME      == test name
## )
###############################################################################

function( Plato_new_test TEST_NAME )

  get_filename_component(testDirName ${CMAKE_CURRENT_SOURCE_DIR} NAME)
  set( TEST_NAME ${testDirName} PARENT_SCOPE )

endfunction( Plato_new_test )

###############################################################################
## Plato_abstract_to_realized( 
## )
###############################################################################

function( Plato_abstract_to_realized ABSOLUTE_PATH_TO_ABSTRACT_FILES BINARY_REALIZED_FILES SED_COMMAND_SUBS)

    # for each abstract and realized file pair
    list(LENGTH BINARY_REALIZED_FILES LIST_LENGTH)
    math(EXPR LAST_LIST_INDEX "${LIST_LENGTH} - 1")
    foreach(LIST_INDEX RANGE ${LAST_LIST_INDEX})
        # get abstract and realized
        list(GET ABSOLUTE_PATH_TO_ABSTRACT_FILES ${LIST_INDEX} THIS_ABSOLUTE_SOURCE)
        list(GET BINARY_REALIZED_FILES ${LIST_INDEX} THIS_RELATIVE_BINARY)

        # make realized from abstract via substitutions
        EXECUTE_PROCESS(COMMAND sed "${SED_COMMAND_SUBS}" ${THIS_ABSOLUTE_SOURCE} OUTPUT_FILE ${CMAKE_CURRENT_BINARY_DIR}/${THIS_RELATIVE_BINARY} RESULT_VARIABLE HAD_ERROR)

        if(HAD_ERROR)
            message(FATAL_ERROR "abstract file substitution failed")
        endif()
    endforeach()

endfunction( Plato_abstract_to_realized )

