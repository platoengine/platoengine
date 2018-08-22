set( testFiles "amgx.json;dfdx.py;computeGradientX.py;InternalEnergyGradX_appfile.xml;InternalEnergyGradX_assoc.txt;InternalEnergyGradX.gen;InternalEnergyGradX_input.xml")
Plato_add_test_files( "${testFiles}" )

## Plato_add_test_files() only works for files.  square_tet_bc.osh is a directory.
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/InternalEnergyGradX.osh DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

Plato_new_test(TEST_NAME)

set( TEST_COMMAND "${PYTHON_EXECUTABLE} computeGradientX.py" )
set( OUTPUT_MESH "InternalEnergyGradX.exo" )

Plato_add_serial_test( ${TEST_COMMAND} ${TEST_NAME} ${OUTPUT_MESH} )
