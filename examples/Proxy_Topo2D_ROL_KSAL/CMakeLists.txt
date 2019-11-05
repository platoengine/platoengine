# specify codes
if(PLATOPROXY)

# Name the test with the directory name
Plato_new_test(TEST_NAME)

# Copy Input files from source to binary dir
set( testFiles "interface.xml;rol_inputs.xml;solution.txt.gold;diff.awk")
Plato_add_test_files("${testFiles}")

# Set number of processors for PlatoMain comm
set( PLATOMAIN_NP 1 )

# Set number of processors for Performer comms
set( PROXY_NP 1 )

# Create list of comm sizes 
set( NUM_PROCS "${PLATOMAIN_NP} ${PROXY_NP}" )

# Specify which comm will provide the output for comparison
set( IO_COMM_INDEX 0 )

# Set the name of the interface file
set( INTERFACE_FILE interface.xml )

# Set file names for diff
set( OUT_FILE "ROL_output.txt")
set( GOLD_FILE "solution.txt.gold")
set( AWK_FILE "diff.awk")

# Create simple regression test and define the PlatoMain performer
Plato_create_simple_test(PLATO_NEW_TEST ${PLATOMAIN_BINARY} ${PLATOMAIN_NP} ${INTERFACE_FILE})

# Define/add a performer
set( LOCAL_COMM_ID 1 )
Plato_add_simple_performer(PLATO_NEW_TEST ${TOPOPROXY_BINARY} ${PROXY_NP} ${LOCAL_COMM_ID} ${INTERFACE_FILE})

# Add the test
Plato_add_awk_test(${PLATO_NEW_TEST} ${TEST_NAME} ${NUM_PROCS} ${IO_COMM_INDEX})

endif()
