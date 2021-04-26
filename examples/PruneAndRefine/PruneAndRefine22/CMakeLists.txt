# specify codes
if(ANALYZE)

# Name the test with the directory name
Plato_new_test(TEST_NAME)

# Copy Input files from source to binary dir
set( testFiles "plato_input_deck.i;pa_comp_min1.exo;restart_data.exo")
Plato_add_test_files("${testFiles}")

# write out the path info in the plato input deck
Plato_add_text_to_file("plato_input_deck.i" "begin paths\n")
Plato_add_text_to_file("plato_input_deck.i" "code PlatoMain ${PLATOMAIN_BINARY}\n")
Plato_add_text_to_file("plato_input_deck.i" "code plato_analyze ${ANALYZE_BINARY}\n")
Plato_add_text_to_file("plato_input_deck.i" "end paths\n")

# Specify which comm will provide the output for comparison
set( IO_COMM_INDEX 0 )

# Set number of processors for PlatoMain comm 
set( PLATOMAIN_NP 1 )

# Set number of processors for Performer comms
set( ANALYZE_NP 1 )

# Create list of comm sizes 
set( NUM_PROCS "${PLATOMAIN_NP} ${ANALYZE_NP}" )

# set mesh for epu at end
set(OUTPUT_MESH platomain.exo)
set(XMLGEN_COMMAND "../../../base/src/input_generator/XMLGenerator plato_input_deck.i")

set( SKIP_EXODIFF 0 )

# Run test
Plato_add_xmlgen_test(${TEST_NAME} ${XMLGEN_COMMAND} ${NUM_PROCS} ${IO_COMM_INDEX} ${OUTPUT_MESH} ${SKIP_EXODIFF})

endif()
