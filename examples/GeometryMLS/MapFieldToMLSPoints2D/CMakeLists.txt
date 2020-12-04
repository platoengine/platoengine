set( testFiles "mapFieldToMLSPoints2D.py;platoApp.xml;platoMain.xml;square_tri_bc.exo;analyzeApp.xml;amgx.json;square_tri_bc.xml")
Plato_add_test_files( "${testFiles}" )

Plato_new_test(TEST_NAME)

set( TEST_COMMAND "${Python2_EXECUTABLE} mapFieldToMLSPoints2D.py" )
set( OUTPUT_MESH "MLS_Field.exo" )

Plato_add_serial_test( ${TEST_COMMAND} ${TEST_NAME} ${OUTPUT_MESH} )
