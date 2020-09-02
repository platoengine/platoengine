set( testFiles "opt.py;platoApp.xml;platoMain.xml;lbracket_2d.exo;analyzeApp.xml;amgx.json;lbracket_2d.xml")
Plato_add_test_files( "${testFiles}" )

Plato_new_test(TEST_NAME)

set( TEST_COMMAND "${Python2_EXECUTABLE} opt.py" )
set( OUTPUT_MESH "LBracket2D.exo" )

Plato_add_serial_test( ${TEST_COMMAND} ${TEST_NAME} ${OUTPUT_MESH} )
