set( testFiles "BoxWithHoleBodyMesh.su2")
Plato_add_test_files( "${testFiles}" )

Plato_new_test(TEST_NAME)

set( TEST_COMMAND "${CMAKE_BINARY_DIR}/base/src/Su2ToExodus/Su2ToExodus BoxWithHoleBodyMesh.su2 output_mesh.exo mark 2 nodeset mark 3 nodeset mark 1 sideset" )
set( OUTPUT_MESH "output_mesh.exo" )

Plato_add_serial_test( ${TEST_COMMAND} ${TEST_NAME} ${OUTPUT_MESH} )
