set( testFiles "input_mesh.exo")
Plato_add_test_files( "${testFiles}" )

Plato_new_test(TEST_NAME)

set( TEST_COMMAND "${CMAKE_BINARY_DIR}/base/src/support_structure/create_support_structure --input_mesh=input_mesh.exo --output_mesh=output_mesh.exo --build_plate_normal='0 1 0' --design_field_name=topology --overhang_angle=45 --algorithm=4" )
set( OUTPUT_MESH "output_mesh.exo" )

Plato_add_serial_test( ${TEST_COMMAND} ${TEST_NAME} ${OUTPUT_MESH} )
