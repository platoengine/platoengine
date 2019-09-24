set( testFiles "amgx.json;dfdx.py;computeGradientX.py;InternalEnergyGradX_appfile.xml;bolted_bracket.exo;InternalEnergyGradX_input.xml")
Plato_add_test_files( "${testFiles}" )

Plato_new_test(TEST_NAME)

set( TEST_COMMAND "${Python2_EXECUTABLE} computeGradientX.py" )
set( OUTPUT_MESH "InternalEnergyGradX.exo" )

Plato_add_serial_test( ${TEST_COMMAND} ${TEST_NAME} ${OUTPUT_MESH} )
