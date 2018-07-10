set( testFiles "mapFieldToMLSPoints.py;platoApp.xml;platoMain.xml;square_tet_bc.gen;alexaApp.xml;amgx.json;square_tet_bc_assoc.txt;square_tet_bc.xml")
Plato_add_test_files( "${testFiles}" )

## Plato_add_test_files() only works for files.  square_tet_bc.osh is a directory.
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/square_tet_bc.osh DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

Plato_new_test(TEST_NAME)

set( TEST_COMMAND "${PYTHON_EXECUTABLE} mapFieldToMLSPoints.py" )
set( OUTPUT_MESH "MLS_Field.exo" )

Plato_add_serial_test( ${TEST_COMMAND} ${TEST_NAME} ${OUTPUT_MESH} )
