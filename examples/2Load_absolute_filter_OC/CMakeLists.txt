# specify codes
if(PLATOSTATICS)

# Name the test with the directory name
Plato_new_test(TEST_NAME)

# Copy Input files from source to binary dir
set( testFiles "interface.xml;lightmp1.xml;lightmp2.xml;localApp1.xml;localApp2.xml;platoApp.xml;platomain.xml;unsmesh.gen;unsmesh.jou")
Plato_add_test_files("${testFiles}")

# Set number of processors for PlatoMain comm
set( PLATOMAIN_NP 3 )

# Set number of processors for Performer comms
set( PLATOSTATICS_NP 4 )

# Create list of comm sizes 
set( NUM_PROCS "${PLATOMAIN_NP} ${PLATOSTATICS_NP} ${PLATOSTATICS_NP}" )

# Specify which comm will provide the output for comparison
set( IO_COMM_INDEX 0 )

# Set the name of the interface file
set( INTERFACE_FILE interface.xml )

# Create the test and define the PlatoMain performer
set(APP_FILE platoApp.xml)
set(INPUT_FILE platomain.xml)
Plato_create_test(PLATO_NEW_TEST ${PLATOMAIN_BINARY} ${PLATOMAIN_NP} ${INTERFACE_FILE} ${APP_FILE} ${INPUT_FILE})

# Define/add a performer
set( LOCAL_COMM_ID 1 )
set(APP_FILE localApp1.xml)
set(INPUT_FILE lightmp1.xml)
Plato_add_performer(PLATO_NEW_TEST ${PLATOSTATICS_BINARY} ${PLATOSTATICS_NP} ${LOCAL_COMM_ID} ${INTERFACE_FILE} ${APP_FILE} ${INPUT_FILE})

# Define/add a performer
set( LOCAL_COMM_ID 2 )
set(APP_FILE localApp2.xml)
set(INPUT_FILE lightmp2.xml)
Plato_add_performer(PLATO_NEW_TEST ${PLATOSTATICS_BINARY} ${PLATOSTATICS_NP} ${LOCAL_COMM_ID} ${INTERFACE_FILE} ${APP_FILE} ${INPUT_FILE})

# Add the test
set(INPUT_MESH unsmesh.gen)
set(OUTPUT_MESH platomain.exo)
Plato_add_test(${PLATO_NEW_TEST} ${TEST_NAME} ${NUM_PROCS} ${IO_COMM_INDEX} ${INPUT_MESH} ${OUTPUT_MESH})

endif()
