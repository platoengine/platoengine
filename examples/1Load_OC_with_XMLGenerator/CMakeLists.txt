# specify codes
if(PLATOSTATICS)

# Name the test with the directory name
Plato_new_test(TEST_NAME)

# Copy Input files from source to binary dir
set( testFiles "plato_input_deck.i;input_mesh.exo")
Plato_add_test_files("${testFiles}")

# Specify which comm will provide the output for comparison
set( IO_COMM_INDEX 0 )

# Set number of processors for PlatoMain comm 
set( PLATOMAIN_NP 1 )

# Set number of processors for Performer comms
set( PLATOSTATICS_NP 1 )

# Create list of comm sizes 
set( NUM_PROCS "${PLATOMAIN_NP} ${PLATOSTATICS_NP}" )

# set mesh for epu at end
set(OUTPUT_MESH platomain.exo)
set(XMLGEN_COMMAND "../../base/src/input_generator/XMLGenerator plato_input_deck.i")

set( SKIP_EXODIFF 0 )

# Run test
Plato_add_xmlgen_test(${TEST_NAME} ${XMLGEN_COMMAND} ${NUM_PROCS} ${IO_COMM_INDEX} ${OUTPUT_MESH} ${SKIP_EXODIFF})

endif()
