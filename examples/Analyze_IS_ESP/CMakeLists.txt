set( testFiles "amgx.json;brick.csm;interface.xml;opt.py;plato_analyze_input_deck_1.xml;plato_analyze_operations_1.xml;platomain.xml;plato_operations.xml")

Plato_add_test_files( "${testFiles}" )

Plato_new_test(TEST_NAME)

set( TEST_COMMAND "${Python2_EXECUTABLE} opt.py" )

Plato_add_single_test( ${TEST_COMMAND} ${TEST_NAME} )
