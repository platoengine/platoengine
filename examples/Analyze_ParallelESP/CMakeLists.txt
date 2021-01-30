# specify codes
if(ANALYZE)

# Name the test with the directory name
Plato_new_test(TEST_NAME)

# Copy Input files from source to binary dir
set( testFiles "amgx.json;defines.xml;interface.xml;plato_analyze_input_deck_1.xml;plato_analyze_operations_1.xml;platoesp_operations.xml;PlatoESP.xml;platomain_operations.xml;PlatoMain.xml;prime.source;rocker.csm")
Plato_add_test_files("${testFiles}")

set( PRETEST_COMMAND "plato-cli geometry esp --input rocker.csm --output-model rocker_opt.csm --output-mesh rocker.exo --tesselation rocker.eto")

# Set number of processors for PlatoMain comm
set( PLATOMAIN_NP 1 )

# Set number of processors for Performer comms
set( PLATOESP_NP 5 )

# Set number of processors for Performer comms
set( ANALYZE_NP 1 )

# Create list of comm sizes 
set( NUM_PROCS "${PLATOMAIN_NP} ${PLATOESP_NP} ${ANALYZE_NP}" )

# Specify which comm will provide the output for comparison
set( IO_COMM_INDEX 0 )

# Set the name of the interface file
set( INTERFACE_FILE interface.xml )

# Create the test and define the PlatoMain performer
set(APP_FILE platomain_operations.xml)
set(INPUT_FILE PlatoMain.xml)
Plato_create_test(PLATO_NEW_TEST ${PLATOMAIN_BINARY} ${PLATOMAIN_NP} ${INTERFACE_FILE} ${APP_FILE} ${INPUT_FILE})

# Define/add a performer
set( LOCAL_COMM_ID 1 )
set(APP_FILE platoesp_operations.xml)
set(INPUT_FILE PlatoESP.xml)
Plato_add_performer(PLATO_NEW_TEST ${PLATOESP_BINARY} ${PLATOESP_NP} ${LOCAL_COMM_ID} ${INTERFACE_FILE} ${APP_FILE} ${INPUT_FILE})

# Define/add a performer
set( LOCAL_COMM_ID 2 )
set(APP_FILE plato_analyze_operations_1.xml)
set(INPUT_FILE --input-config=plato_analyze_input_deck_1.xml)
Plato_add_performer(PLATO_NEW_TEST ${ANALYZE_BINARY} ${ANALYZE_NP} ${LOCAL_COMM_ID} ${INTERFACE_FILE} ${APP_FILE} ${INPUT_FILE})

# prepend pretest command
set( PLATO_NEW_TEST ${PRETEST_COMMAND} ${PLATO_NEW_TEST} )
message("WITH PRETEST: ${PLATO_NEW_TEST}")

# Add the test
set(INPUT_MESH SKIP_DECOMP)
set(OUTPUT_MESH SKIP_EXODIFF)
Plato_add_test("${PLATO_NEW_TEST}" ${TEST_NAME} ${NUM_PROCS} ${IO_COMM_INDEX} ${INPUT_MESH} ${OUTPUT_MESH})

endif()
