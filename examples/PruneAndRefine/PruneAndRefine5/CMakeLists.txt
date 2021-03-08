set( INPUT_MESH "lantern.exo" )
set( OUTPUT_MESH "lantern_pruned.exo" )
set( FIELD_NAME "Rho_node" )
set( NUM_PROCS 4 )
set( NUM_REFINES 0 )
set( PRUNE_MESH 1 )
set( NUM_BUFFER_LAYERS 0 )
set( IO_COMM_INDEX 0 )

set( testFiles "${INPUT_MESH}")
Plato_add_test_files( "${testFiles}" )

Plato_new_test(TEST_NAME)



set( RUN_COMMAND "mpirun -n ${NUM_PROCS} ${CMAKE_BINARY_DIR}/base/src/prune/prune_and_refine --mesh_with_variable=${INPUT_MESH} --mesh_to_be_pruned=${INPUT_MESH} --result_mesh=${OUTPUT_MESH} --field_name=${FIELD_NAME} --number_of_refines=${NUM_REFINES} --number_of_buffer_layers=${NUM_BUFFER_LAYERS} --prune_mesh=${PRUNE_MESH}" )

#Plato_add_serial_test( ${TEST_COMMAND} ${TEST_NAME} ${OUTPUT_MESH} )
Plato_add_test(${RUN_COMMAND} ${TEST_NAME} ${NUM_PROCS} ${IO_COMM_INDEX} ${INPUT_MESH} ${OUTPUT_MESH})

