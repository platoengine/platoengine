set( testFiles "initializeMLSPoints.py;platoApp.xml;platoMain.xml;square_tet_bc.gen")
Plato_add_test_files( "${testFiles}" )

Plato_new_test(TEST_NAME)

set( TEST_COMMAND "${PYTHON_EXECUTABLE} initializeMLSPoints.py" )
set( OUTPUT_MESH "MLS_Field.exo" )

Plato_add_serial_test( ${TEST_COMMAND} ${TEST_NAME} ${OUTPUT_MESH} )
