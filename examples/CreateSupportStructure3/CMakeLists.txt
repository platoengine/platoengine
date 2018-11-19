set( testFiles "input_mesh.exo")
Plato_add_test_files( "${testFiles}" )

Plato_new_test(TEST_NAME)

set( TEST_COMMAND "mpirun -n 1 ${CMAKE_BINARY_DIR}/base/src/support_structure/create_support_structure --input_mesh=input_mesh.exo --output_mesh=output_mesh.exo --design_field_name=density --build_plate_normal='0 1 0' --overhang_angle=60 --algorithm=4" )
set( OUTPUT_MESH "output_mesh.exo" )

Plato_add_serial_test( ${TEST_COMMAND} ${TEST_NAME} ${OUTPUT_MESH} )
