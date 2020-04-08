# specify codes
if(ANALYZE)

# Name the test with the directory name
Plato_new_test(TEST_NAME)

# Copy Input files from source to binary dir
set( testFiles "interface.xml;dynamicsInput.xml;staticsInput.xml;analyzeApp1.xml;analyzeApp2.xml;platoApp.xml;platoMain.xml;amgx.json;design_vol.exo")
Plato_add_test_files("${testFiles}")

# Set number of processors for PlatoMain comm
set( PLATOMAIN_NP 1 )

# Set number of processors for Performer comms
set( ANALYZE_1_NP 1 )
set( ANALYZE_2_NP 1 )

# Create list of comm sizes 
set( NUM_PROCS "${PLATOMAIN_NP} ${ANALYZE_1_NP} ${ANALYZE_2_NP}" )

# Specify which comm will provide the output for comparison
set( IO_COMM_INDEX 0 )

# Set the name of the interface file
set( INTERFACE_FILE interface.xml )

# Create the test and define the PlatoMain performer
set(APP_FILE platoApp.xml)
set(INPUT_FILE platoMain.xml)
Plato_create_test(PLATO_NEW_TEST ${PLATOMAIN_BINARY} ${PLATOMAIN_NP} ${INTERFACE_FILE} ${APP_FILE} ${INPUT_FILE})

# Define/add a performer
set( LOCAL_COMM_ID 1 )
set(APP_FILE analyzeApp1.xml)
set(INPUT_FILE --input-config=dynamicsInput.xml)
Plato_add_performer(PLATO_NEW_TEST ${ANALYZE_BINARY} ${ANALYZE_1_NP} ${LOCAL_COMM_ID} ${INTERFACE_FILE} ${APP_FILE} ${INPUT_FILE})

# Define/add a performer
set( LOCAL_COMM_ID 2 )
set(APP_FILE analyzeApp2.xml)
set(INPUT_FILE --input-config=staticsInput.xml)
Plato_add_performer(PLATO_NEW_TEST ${ANALYZE_BINARY} ${ANALYZE_2_NP} ${LOCAL_COMM_ID} ${INTERFACE_FILE} ${APP_FILE} ${INPUT_FILE})

# Add the test
set(INPUT_MESH design_vol.exo)
set(OUTPUT_MESH platomain.exo)
Plato_add_test(${PLATO_NEW_TEST} ${TEST_NAME} ${NUM_PROCS} ${IO_COMM_INDEX} ${INPUT_MESH} ${OUTPUT_MESH})

endif()
