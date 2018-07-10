set( testFiles "opt.py;platoApp.xml;platoMain.xml;lbracket_2d.gen;alexaApp.xml;amgx.json;lbracket_2d_assoc.txt;lbracket_2d.xml")
Plato_add_test_files( "${testFiles}" )

## Plato_add_test_files() only works for files.  square_tet_bc.osh is a directory.
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/lbracket_2d.osh DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

Plato_new_test(TEST_NAME)

set( TEST_COMMAND "${PYTHON_EXECUTABLE} opt.py" )
set( OUTPUT_MESH "Mitchell2D.exo" )

Plato_add_serial_test( ${TEST_COMMAND} ${TEST_NAME} ${OUTPUT_MESH} )
